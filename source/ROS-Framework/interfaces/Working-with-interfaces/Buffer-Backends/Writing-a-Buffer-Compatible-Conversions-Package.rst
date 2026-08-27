Writing a ``rosidl::Buffer`` compatible conversions package
===========================================================

**Goal:** Learn how to create a ``*_conversions`` package that maps between a
ROS 2 message containing ``uint8[]`` payload fields and a library-specific
native type, while preserving compatibility with ``rosidl::Buffer`` storage
backends.

**Tutorial level:** Advanced

**Time:** 30+ minutes

.. contents:: Table of Contents
   :local:

Introduction
------------

``rosidl::Buffer`` lets generated C++ messages store selected variable-length
primitive array fields, such as ``uint8[]``, in externally managed storage.
With the default CPU backend, the field behaves like ``std::vector<uint8_t>``.
With another negotiated backend, the same field can hold storage such as CUDA
memory.

A conversions package is a user-level library that adapts a ROS message to a
framework-native type.
It does not implement a new transport and does not register a
``rosidl::BufferBackend`` plugin.
Instead, it knows how to interpret one or more ``uint8[]`` fields in a message
and expose them as an object that is natural for a particular library.

Examples include:

* a tensor conversions package that maps
  ``tensor_msgs/msg/ExperimentalTensor`` to an ONNX Runtime tensor, NumPy
  array, CuPy array, or another framework tensor;
* an image conversions package that maps ``sensor_msgs/msg/Image`` to a
  library-specific image view;
* a point cloud conversions package that maps ``sensor_msgs/msg/PointCloud2``
  to a framework-specific point cloud or table representation.

The purpose is interoperability.
Nodes can exchange the same ROS message type on the wire, while each process
uses a conversions package to work with its own in-process data type.
The underlying bytes can be CPU-backed or use another storage backend, subject
to what the conversions package and the selected backend support.

When to write a conversions package
-----------------------------------

Write a conversions package when:

* the ROS message schema already describes the data you want to exchange;
* one or more ``uint8[]`` fields should be exposed as a library-native object;
* the library can consume CPU storage, or can consume a supported non-CPU
  backend such as ``cuda``;
* you do not need a new memory transport.

Do **not** write a new ``rosidl::BufferBackend`` just to add application-level
metadata such as image format, tensor shape, point-field layout, or
framework-specific type names.
Backends are for storage and transport.
Conversions packages are for adapting existing ROS message schemas to
framework-specific APIs.

Package shape
-------------

A typical conversions package is named after the library it supports:
``torch_conversions``, ``onnx_conversions``, ``numpy_conversions``, and so on.
It usually contains:

* a dependency on the message package whose ``uint8[]`` field is being adapted;
* a dependency on ``rosidl_buffer`` when C++ code needs to inspect the field's
  active backend or pass it to backend APIs;
* dependencies on the target library;
* optional dependencies on storage backends, such as ``cuda_buffer``, when the
  package can produce or consume backend-native storage directly;
* tests that round-trip metadata, storage size, and payload contents.

The package does not add a backend name.
Subscribers opt in to the underlying storage backend, for example ``"cuda"``
or ``"cpu"``, not to the conversions package.

Generic API pattern
-------------------

The exact names and return types depend on the message and target library, but
most conversions packages provide four operations.
The examples below use placeholder names for a hypothetical
``mytype_conversions`` package.

Allocate or initialize a message
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Provide a helper that fills message metadata and sizes the relevant ``uint8[]``
storage:

.. code-block:: c++

    MyMsg allocate_msg(
      const MyShapeOrLayout & layout,
      MyElementType element_type,
      std::optional<DeviceKind> device = std::nullopt);

For CPU storage, resize the message field normally.
For a supported non-CPU storage backend, construct a backend-backed
``rosidl::Buffer<uint8_t>`` and assign it to the payload field.
If the requested device or backend is not supported, fail clearly rather than
silently publishing a message whose storage is different from what the caller
requested.

Create a writable native view
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Provide a publisher-side function that returns a writable native view over the
message storage:

.. code-block:: c++

    MyNativeType from_output_msg(MyMsg & msg);

Use this when the application wants to fill the message in place before
publishing.
For CPU-backed fields, the view can point at the field's ``data()`` pointer.
For CUDA-backed fields, use the CUDA buffer API to get a write handle and keep
that handle alive for at least as long as the native view can write through the
pointer.

Create a readable native view or copy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Provide a subscriber-side function that returns either a read-only view or an
independent copy:

.. code-block:: c++

    MyNativeType from_input_msg(const MyMsg & msg, bool clone = true);

The ``clone=true`` default is a useful safety policy because a subscriber can
mutate the returned object without depending on the message lifetime.
If you also offer ``clone=false``, document that the returned view aliases the
message's storage and must be treated as read-only unless your library can
prove exclusive ownership.

For non-CPU backends, use the backend's input/read API and keep the read handle
alive for the lifetime of the native view.
If your target library cannot consume a backend, either request CPU-backed
subscriptions in your examples or copy through CPU explicitly and document the
cost.

Copy an existing native object into a message
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Provide a helper that copies an existing native object into preallocated
message storage and refreshes metadata:

.. code-block:: c++

    void to_msg(MyMsg & msg, const MyNativeType & value);

This helper should:

* normalize or preserve the source layout intentionally;
* verify that the payload field is large enough;
* copy into CPU or backend-native storage as appropriate;
* update all message metadata consistently with the copied payload.

Tensor messages
---------------

``tensor_msgs/msg/ExperimentalTensor`` is the first common tensor message type
built around this pattern.
It carries DLPack-aligned metadata (dtype, shape, strides, and byte offset)
plus a ``uint8[] data`` field.
In generated C++ code, that ``data`` field is a
``rosidl::Buffer<uint8_t>``, so tensor bytes can use CPU storage or another
negotiated backend such as ``cuda``.

The message schema is:

.. code-block:: text

    uint8  dtype_code
    uint8  dtype_bits
    uint16 dtype_lanes

    int64[] shape
    int64[] strides
    uint64  byte_offset

    uint8[] data

Device placement is not stored as a separate message field.
It is derived from the storage backend of ``msg.data``.
For example, ``msg.data.get_backend_type() == "cpu"`` indicates ordinary host
memory, while ``"cuda"`` indicates CUDA-backed storage when the CUDA backend is
installed and negotiated.

Reference: ``torch_conversions``
--------------------------------

``torch_conversions`` is the first conversions package for
``tensor_msgs/msg/ExperimentalTensor``.
It maps the common tensor message to and from ``at::Tensor`` without
registering a separate buffer backend.

At a high level, it provides:

* ``allocate_tensor_msg()`` to fill DLPack metadata and allocate
  ``ExperimentalTensor.data`` with CPU or CUDA-backed storage;
* ``from_output_tensor_msg()`` to create a writable ``at::Tensor`` view over a
  publisher-owned message;
* ``from_input_tensor_msg()`` to create a subscriber-side ``at::Tensor`` copy
  or read-only view;
* ``to_tensor_msg()`` to copy an existing ``at::Tensor`` into preallocated
  message storage and refresh metadata;
* DLPack import/export helpers so PyTorch can consume the message storage
  through its existing DLPack support.

Because ``torch_conversions`` uses the common
``tensor_msgs/msg/ExperimentalTensor`` schema, a publisher using PyTorch can
communicate with a subscriber using a different tensor library, as long as that
subscriber has its own conversions package for the same message schema and can
consume the selected storage backend.

Creating a tensor-compatible conversions package
------------------------------------------------

To create a conversions package for another tensor library, follow the generic
API pattern above with ``tensor_msgs/msg/ExperimentalTensor`` as the message
type.
The examples below use placeholder names for a hypothetical
``mytensor_conversions`` package.

Allocate a tensor message
^^^^^^^^^^^^^^^^^^^^^^^^^

Provide a helper that fills tensor metadata and sizes ``msg.data``:

.. code-block:: c++

    tensor_msgs::msg::ExperimentalTensor allocate_tensor_msg(
      const std::vector<int64_t> & shape,
      MyTensorDType dtype,
      std::optional<DeviceKind> device = std::nullopt);

For CPU storage, resize ``msg.data`` normally.
For a supported non-CPU storage backend, construct a backend-backed
``rosidl::Buffer<uint8_t>`` and assign it to ``msg.data``.

Map metadata
^^^^^^^^^^^^

Map the target library's dtype to
``{dtype_code, dtype_bits, dtype_lanes}``.
If a dtype cannot be represented, throw a clear unsupported-dtype error.

``shape`` stores the tensor dimensions.
``strides`` stores element strides; an empty strides array follows the DLPack
convention for contiguous row-major storage.
``byte_offset`` points to the first logical element inside ``data`` and allows
zero-copy views into a larger allocation.

Use DLPack when available
^^^^^^^^^^^^^^^^^^^^^^^^^

If the target library supports DLPack, prefer DLPack import/export over a
library-specific metadata encoding.
DLPack already describes dtype, shape, strides, byte offset, device type, and
data pointer ownership conventions.
Your conversions package can translate ``ExperimentalTensor`` to a
``DLManagedTensor`` and let the framework consume it.

When building a DLPack view:

* store shape and stride arrays in memory that outlives the
  ``DLManagedTensor``;
* keep any backend read or write handle alive with the DLPack context;
* set the DLPack device from ``msg.data.get_backend_type()``;
* provide a deleter that releases all context objects after the framework is
  done with the tensor.

Publish and subscribe
^^^^^^^^^^^^^^^^^^^^^

A publisher should publish ``tensor_msgs/msg/ExperimentalTensor`` directly:

.. code-block:: c++

    auto msg = mytensor_conversions::allocate_tensor_msg(
      {height, width, channels}, mytensor::uint8, DeviceKind::CUDA);
    mytensor::Tensor output = mytensor_conversions::from_output_tensor_msg(msg);
    render(output);
    publisher_->publish(msg);

A subscriber opts in to the underlying storage backend and then converts the
message:

.. code-block:: c++

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.acceptable_buffer_backends = "cuda";

    subscription_ = this->create_subscription<tensor_msgs::msg::ExperimentalTensor>(
      "tensor", 10,
      [](const tensor_msgs::msg::ExperimentalTensor::SharedPtr msg) {
        mytensor::Tensor input =
          mytensor_conversions::from_input_tensor_msg(*msg, /*clone=*/false);
        run_model(input);
      },
      sub_opts);

If the subscriber cannot consume CUDA-backed storage, set
``acceptable_buffer_backends`` to ``"cpu"`` or omit the option.
The same ``ExperimentalTensor`` schema is used either way.

Testing checklist
-----------------

Before releasing a conversions package, test:

* metadata mapping in both directions, including unsupported metadata errors;
* scalar, vector, matrix, and higher-rank tensor shapes when using
  ``ExperimentalTensor``;
* contiguous and strided payloads;
* nonzero offsets or view-like layouts, if supported by the message type;
* CPU-backed round trips;
* each non-CPU backend the package claims to support;
* copy and zero-copy view lifetimes;
* publisher/subscriber tests with ``rmw_fastrtps_cpp`` and
  ``acceptable_buffer_backends`` set to the backend names your package
  supports.

Where to go next
----------------

* :doc:`About-Buffer-Backends` -- conceptual
  background on ``rosidl::Buffer`` and backend negotiation.
* :doc:`Using-Buffer-Backends` -- user-facing examples for
  publishing and subscribing with buffer-backed tensor messages.
* :doc:`Writing-a-Buffer-Backend` -- implementer guide for creating a new
  storage backend when an existing backend is not enough.
