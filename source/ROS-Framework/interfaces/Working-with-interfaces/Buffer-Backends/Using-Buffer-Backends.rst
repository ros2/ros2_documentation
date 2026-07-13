Using ``rosidl::Buffer`` backends
=================================

.. contents:: Contents
   :depth: 2
   :local:

Overview
--------

``rosidl::Buffer<T>`` is the generated C++ representation of variable-length
primitive array fields in ROS 2 messages (``uint8[]``, ``float32[]``, ...).
It behaves like a ``std::vector<T>`` by default, but its storage is pluggable
so that vendors can back those fields with non-CPU memory (for example, GPU
memory) and transport them with as few copies as the RMW and the backend
allow.

See :doc:`About-Buffer-Backends` for a conceptual
overview.

This guide covers:

* how to keep existing publishers and subscribers working without changes;
* how to opt a subscription in to a specific set of backends;
* how to use a backend's user-facing API to read and write its native data
  type;
* how ``tensor_msgs/msg/ExperimentalTensor`` and ``torch_conversions`` use the
  same backend mechanism for tensor data;
* how to make sure publisher and subscriber are configured compatibly.

Default behavior (no changes required)
--------------------------------------

If no backend is explicitly configured, ``rosidl::Buffer<T>`` still behaves
like ``std::vector<T>``.
Existing code such as

.. code-block:: c++

    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->data.resize(width * height * 3);
    std::memcpy(msg->data.data(), source_ptr, msg->data.size());
    publisher_->publish(std::move(msg));

continues to compile and run unchanged because ``rosidl::Buffer<uint8_t>``
implicitly converts to ``std::vector<uint8_t> &`` when its active backend is
CPU.

On the subscription side, if a subscription does not opt in to any
non-CPU backend (see below), the RMW delivers the data with the CPU backend
just like it did before the feature was introduced, so
``msg->data.size()``, ``msg->data[i]``, ``msg->data.data()`` and so on keep
working.

RMW support
-----------

Buffer backends require support from the RMW implementation to negotiate
and serialize descriptors on the wire.

The first supported RMW integration is ``rmw_fastrtps_cpp``.
With other RMW implementations, a subscription that requests a non-CPU
backend still functions -- it simply receives CPU-backed data, exactly as if
``acceptable_buffer_backends`` had not been set.

The non-CPU backend path currently applies to topic publish/subscribe only.
Services and actions do not expose an ``acceptable_buffer_backends`` option and continue to use their normal request, response, feedback, status, and result serialization paths.

Discovering installed backends
------------------------------

Backend plugins are discovered through ``pluginlib`` and live in packages
whose ``package.xml`` exports the ``rosidl_buffer_backend`` plugin
description file.
To see which backends are installed in the current environment:

.. code-block:: console

    $ ros2 pkg list | grep _buffer_backend

Each backend package usually installs two things:

* the backend plugin itself (a shared library), registered under a short
  name such as ``cuda`` in the plugin XML;
* a descriptor-message package (``*_msgs``) containing the backend's
  descriptor ``.msg`` type.

Both must be installed on every node that participates in the topic --
publishers and subscribers.

Enabling a backend on a subscription
------------------------------------

Publishers do **not** advertise a list of backends; they simply publish
whatever backend their buffer currently uses.
Subscriptions, on the other hand, opt in to non-CPU backends via the
``acceptable_buffer_backends`` option on ``rclcpp::SubscriptionOptions``
(or the ``acceptable_buffer_backends`` keyword argument in ``rclpy``).

The option is a comma-separated string of backend names.
It accepts the following forms:

.. list-table::
   :widths: 25 75
   :header-rows: 1

   * - Value
     - Meaning
   * - empty or ``"cpu"``
     - CPU only.
       This is the default and preserves backward compatibility.
   * - ``"any"``
     - Accept any backend that is installed in this process.
   * - ``"cuda,mydev"``
     - Accept only the listed backends, in addition to CPU.
       Names match the plugin ``name`` attribute in the backend's plugin XML.

CPU is always implicitly acceptable, so no matter what is specified, a
subscription can always receive CPU-backed messages (for example, when the
publisher's backend cannot serve this particular peer).

C++ example
^^^^^^^^^^^

.. code-block:: c++

    #include <cuda_runtime.h>

    #include "rclcpp/rclcpp.hpp"
    #include "sensor_msgs/msg/image.hpp"
    #include "cuda_buffer/cuda_buffer_api.hpp"

    class GpuImageSubscriber : public rclcpp::Node
    {
    public:
      GpuImageSubscriber()
      : Node("gpu_image_subscriber")
      {
        cudaStreamCreate(&stream_);

        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.acceptable_buffer_backends = "cuda";

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
          "image", 10,
          [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
            if (msg->data.get_backend_type() == "cuda") {
              // Zero-copy GPU path: read the device pointer directly.
              auto rh = cuda_buffer_backend::from_input_buffer(msg->data, stream_);
              process_on_gpu(rh.get_ptr(), msg->data.size(), stream_);
            } else {
              // CPU fallback: msg->data behaves like std::vector<uint8_t>.
              process_on_cpu(msg->data.data(), msg->data.size());
            }
          },
          sub_opts);
      }

    private:
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
      cudaStream_t stream_{nullptr};
    };

Two things to note:

* The subscription only declares which backends are *acceptable*; the actual
  backend of a received message is whatever the publisher sent, subject to
  negotiation.
  Always branch on ``msg->data.get_backend_type()`` before using a
  backend-specific API.
* Backend APIs (``cuda_buffer_backend::from_input_buffer``,
  ``cuda_buffer_backend::from_output_buffer``, ...) are provided by the backend
  package, not by ``rclcpp``.
  Consult each backend's own documentation for its full API surface.

Python
^^^^^^

``rclpy`` exposes the same option as a keyword argument on
``Node.create_subscription``:

.. code-block:: python

    self.create_subscription(
        Image,
        'image',
        self.image_callback,
        10,
        acceptable_buffer_backends='cuda')

On the Python side the ``data`` field still surfaces as a byte sequence;
backend user-facing APIs are currently C++-only, so Python subscribers that
accept non-CPU backends typically rely on the backend's own CPU-fallback
path (the RMW serializes the buffer to CPU if the backend cannot serve the
Python endpoint directly).

Publisher side
--------------

Publishers use the backend's allocation API to produce a message whose
``uint8[]`` field is already backed by that backend's memory.
For example, with the CUDA backend:

.. code-block:: c++

    #include "cuda_buffer/cuda_buffer_api.hpp"

    sensor_msgs::msg::Image msg;
    msg.data = cuda_buffer_backend::allocate_buffer(width * height * 3);
    msg.header.stamp = this->now();
    msg.width = width;
    msg.height = height;
    msg.encoding = "rgb8";
    msg.step = width * 3;

    {
      auto wh = cuda_buffer_backend::from_output_buffer(msg.data, stream_);
      my_kernel<<<grid, block, 0, stream_>>>(wh.get_ptr(), ...);
    }

    publisher_->publish(msg);

A publisher does not need any ``rclcpp`` option changes; the backend of the
buffer inside the message is what the RMW sees at publish time.
If a given subscriber has not opted in to that backend, the RMW falls back
to CPU serialization for that peer transparently -- the publisher writes the
same message either way.

QoS considerations
------------------

QoS compatibility is still checked by the RMW implementation in the usual way.
The backend selection controls how eligible buffer fields are represented once a sample is being delivered; it does not replace DDS reliability, history, deadline, lifespan, or liveliness behavior.

The CUDA backend's zero-copy path is intended for live topic samples.
Use volatile durability for CUDA-backed topics.
Transient-local durability for late-joining subscribers is not a supported zero-copy CUDA use case, because the descriptor refers to publisher-owned live memory that may be recycled by the backend.

Tensor messages with ``torch_conversions``
------------------------------------------

The experimental tensor path uses a normal ROS 2 message,
``tensor_msgs/msg/ExperimentalTensor``.
Its ``data`` field is a ``rosidl::Buffer<uint8_t>``, while the other fields
carry DLPack-aligned tensor metadata such as dtype, shape, strides, and byte
offset.
The ``torch_conversions`` package is a header-only helper library that fills
that message and converts it to or from ``at::Tensor``; it does not register a
separate buffer backend.
Framework-specific packages can follow the same pattern for other tensor
libraries.
See :doc:`Writing-a-Buffer-Compatible-Conversions-Package`
for guidance on writing a ``*_conversions`` package.

On the publisher side:

.. code-block:: c++

    #include "torch_conversions/torch_conversions.hpp"
    #include "tensor_msgs/msg/experimental_tensor.hpp"

    auto guard = torch_conversions::set_stream();
    auto msg = torch_conversions::allocate_tensor_msg(
      {height, width, 4}, torch::kByte, c10::kCUDA);
    torch_conversions::to_tensor_msg(msg, rendered_frame);
    publisher_->publish(msg);

On the subscriber side, opt in to the storage backend you want to accept and
then convert the received tensor message:

.. code-block:: c++

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.acceptable_buffer_backends = "cuda";

    subscription_ = this->create_subscription<tensor_msgs::msg::ExperimentalTensor>(
      "image", 10,
      [](const tensor_msgs::msg::ExperimentalTensor::SharedPtr msg) {
        auto guard = torch_conversions::set_stream();
        at::Tensor frame =
          torch_conversions::from_input_tensor_msg(*msg, /*clone=*/false);
        consume(frame);
      },
      sub_opts);

Ensuring a compatible pub/sub pair
----------------------------------

Aside from matching backend names (``"cuda"`` on both sides, and so on),
there are three practical rules for making a non-CPU path actually work
end-to-end.

1. Install the same backend on both sides
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Both the backend plugin package **and** its descriptor-message package
(``*_backend_msgs``) must be installed and available to the process.
If the subscriber is missing either, it will silently fall back to CPU and
log a warning from the RMW buffer-backend loader.

Check with:

.. code-block:: console

    $ ros2 pkg list | grep cuda_buffer

Libraries built on top of ``rosidl::Buffer`` may have their own dependencies.
For example, ``torch_conversions`` uses the ``cuda`` backend for CUDA tensor
storage when the CUDA packages are available, but it does not register a
separate buffer backend name.

2. Use the same RMW on both sides
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use the same RMW implementation for publisher and subscriber.
Buffer descriptors are serialized through the RMW's normal type-support
pipeline, so the ordinary advice for
:doc:`multiple RMW implementations <../../../../Get-Started/Installation/RMW-Implementations/Working-with-multiple-RMW-implementations>`
applies: set ``RMW_IMPLEMENTATION`` consistently.

3. Keep backend versions aligned with ROS 2 core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Backend plugins are ABI-coupled to ``rosidl_buffer_backend`` and to the
RMW's type-support library.
Rebuild installed backends after upgrading ROS 2 core, the same way you
would rebuild any other plugin package.

Transport scope depends on the backend
--------------------------------------

``rosidl::Buffer`` only guarantees that the descriptor round-trips through
the RMW.
Whether a given ``backend`` can actually carry data intra-process,
inter-process on the same host, or inter-host is entirely a property of the
backend implementation.
Consult each backend's own documentation for its support matrix.

For reference, the CUDA backend currently supports:

* intra-process (same Python/C++ process);
* inter-process on the same host, same GPU, same user (via CUDA VMM IPC);
* inter-host CUDA transport is not currently supported by this backend, so it declines that peer and the publish/subscribe path uses CPU serialization for the field.

Other backends have their own constraints, documented in their own
repositories.

Diagnostics
-----------

Inspecting negotiated transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Inside a subscription callback, ``msg->data.get_backend_type()`` returns the
backend of the just-received message (``"cpu"``, ``"cuda"``, ...).
Comparing it to what you expected is the quickest way to tell whether the
zero-copy path actually engaged or the RMW fell back to CPU.

Listing loaded backends at runtime
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

From C++, the registry exposes a list of discovered plugins:

.. code-block:: c++

    #include "rosidl_buffer_backend_registry/buffer_backend_registry.hpp"

    rosidl_buffer_backend_registry::BufferBackendRegistry registry;
    for (const auto & name : registry.get_backend_names()) {
      RCLCPP_INFO(rclcpp::get_logger("app"), "Loaded buffer backend: %s", name.c_str());
    }

Most backends also log a warning when they fall back to CPU
serialization, which is visible at the default log level.

Interaction with other transport features
-----------------------------------------

* :doc:`Loaned messages <../../../client-libraries/Working-with-Client-Libraries/Configure-ZeroCopy-loaned-messages>` operate at a
  different layer: they let the RMW own the *message* memory, while
  ``rosidl::Buffer`` backends control the storage of individual variable-length
  array fields.
  The two features can be combined when both the RMW and the backend support
  it.
* :doc:`Intra-process communication <../../../nodes/Working-with-nodes/intra-process/Intra-Process-Communication>`
  is orthogonal: a backend may implement its own intra-process fast path
  (the CUDA backend does), but the decision is up to the backend.
