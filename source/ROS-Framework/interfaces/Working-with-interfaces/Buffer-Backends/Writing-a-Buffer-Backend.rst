Writing a ``rosidl::Buffer`` backend
====================================

**Goal:** Learn how to implement and package a new ``rosidl::Buffer``
backend plugin so that a vendor-specific memory domain (GPU, shared memory,
accelerator memory, ...) can back ``uint8[]`` fields of ROS 2 messages.

**Tutorial level:** Advanced

**Time:** 60+ minutes

.. contents:: Table of Contents
   :local:

Introduction
------------

:doc:`rosidl::Buffer backends <About-Buffer-Backends>`
let vendors transport the bytes of a ``uint8[]`` message field through ROS 2
pub/sub with as few copies as their memory technology permits.
A backend is a ``pluginlib`` plugin that implements the
``rosidl::BufferBackend`` interface from the ``rosidl_buffer_backend``
package.

This guide shows how to build such a plugin, end to end.
It uses a hypothetical ``mydev`` backend in the prose (for a vendor device
memory domain) and refers to the real CUDA backend, the ``torch_conversions``
helper library, and the demo backend in the reference links for concrete code.

This guide is intended as a starting point; the existing reference backends
go into much more detail than is practical to reproduce here.
The core contract, however, is small: once your plugin can answer the six
``BufferBackend`` virtuals correctly for a single message type, everything
else is polish.

When **not** to write a backend
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A backend is the right tool when you want to change the *memory domain* of variable-length primitive array fields.
If your goal is to reduce copies within CPU memory, you are almost certainly looking for :doc:`intra-process communication </ROS-Framework/nodes/Working-with-nodes/intra-process/Intra-Process-Communication>` or :doc:`loaned messages <../../../client-libraries/Working-with-Client-Libraries/Configure-ZeroCopy-loaned-messages>` instead.

Reference implementations
^^^^^^^^^^^^^^^^^^^^^^^^^

Two open-source backends and one user-level tensor library are good reading.
All are small enough to read in full:

* ``cuda_buffer_backend``: a realistic CUDA backend built on CUDA VMM
  and CUDA IPC, with intra-process, inter-process same-host, and CPU-fallback
  paths.
  See `ros2/rosidl_buffer_backends <https://github.com/ros2/rosidl_buffer_backends>`__.
* ``torch_conversions``: a header-only user library, not a backend plugin.
  It converts between ``tensor_msgs/msg/ExperimentalTensor`` and ``at::Tensor``.
  The tensor message's ``uint8[] data`` field is transported by an ordinary
  buffer backend such as ``cuda`` or ``cpu``.
  Same repository.

The ``BufferBackend`` interface
-------------------------------

The plugin interface is declared in
`rosidl_buffer_backend/buffer_backend.hpp
<https://github.com/ros2/rosidl/blob/{DISTRO}/rosidl_buffer_backend/include/rosidl_buffer_backend/buffer_backend.hpp>`__.
It has six methods you will commonly override, plus a descriptor-size
constant.

.. code-block:: c++

    namespace rosidl
    {

    inline constexpr size_t kMaxBufferDescriptorSize = 4096;

    class BufferBackend
    {
    public:
      virtual ~BufferBackend() = default;

      // Identity
      virtual std::string get_backend_type() const = 0;
      virtual std::string get_backend_metadata() const { return ""; }

      // Descriptor type support and construction
      virtual const rosidl_message_type_support_t *
      get_descriptor_type_support() const = 0;
      virtual std::shared_ptr<void> create_empty_descriptor() const = 0;
      virtual std::shared_ptr<void> create_descriptor_with_endpoint(
        const void * impl,
        const rmw_topic_endpoint_info_t & endpoint_info) const = 0;
      virtual std::unique_ptr<void, void (*)(void *)>
      from_descriptor_with_endpoint(
        const void * descriptor,
        const rmw_topic_endpoint_info_t & endpoint_info) const = 0;

      // Discovery hooks (optional to override)
      virtual void on_creating_endpoint(
        const rmw_topic_endpoint_info_t & endpoint_info) const;
      virtual std::pair<bool, std::vector<std::set<uint32_t>>>
      on_discovering_endpoint(
        const rmw_topic_endpoint_info_t & endpoint_info,
        const std::vector<rmw_topic_endpoint_info_t> & existing_endpoints,
        const std::unordered_map<std::string, std::string> & endpoint_supported_backends);
    };

    }  // namespace rosidl

Anatomy of a backend package
----------------------------

A complete backend is usually split across three or four ROS 2 packages:

#. **Core memory-domain library** (e.g. ``mydev_buffer``): the non-plugin
   code -- custom allocator, ``BufferImplBase<T>`` subclass, IPC manager,
   RAII handles.
   This is where most of the vendor's engineering lives.
#. **Descriptor message package** (e.g. ``mydev_buffer_backend_msgs``): a
   normal ROS 2 ``.msg`` that describes how to reconstruct the buffer on
   the receiving side.
#. **Backend plugin package** (e.g. ``mydev_buffer_backend``): a thin
   shared library that implements ``rosidl::BufferBackend`` and is
   registered via ``pluginlib``.
#. **User-facing API header** (optional, usually shipped inside the core
   library): helpers like ``allocate_buffer``, ``from_output_buffer``,
   ``from_input_buffer``, and ``to_buffer`` that application authors use to
   produce and consume backend-native data.

Step 1: Design the descriptor message
-------------------------------------

The descriptor is the only thing that actually travels over the wire.
It must contain everything the receiving endpoint needs to reconstruct a
buffer whose size and contents match the publisher's buffer.

Two important constraints:

* The serialized descriptor must be no larger than
  ``rosidl::kMaxBufferDescriptorSize`` bytes (4096).
  The RMW plans its serialization buffers around this bound.
* The descriptor package must be a plain ``rosidl_default_generators``
  message package so that the generic
  ``rosidl_typesupport_cpp::get_message_type_support_handle<T>()`` returns an
  aggregate handle that contains the FastRTPS C++ sub-handle.
  (Today that sub-handle is the one the RMW layer extracts; see
  :ref:`writing-buffer-backend_type-support`.)

A descriptor typically carries either a small reference that the receiving side uses to re-attach to the payload, a serialized copy of the payload for the CPU-fallback case, or both.
The exact re-attachment mechanism is backend-specific.
For example, a simplified device-IPC descriptor might look like:

.. code-block:: text

    # mydev_buffer_backend_msgs/msg/MyDevBufferDescriptor.msg
    uint64 size                       # allocation size in bytes
    int32  device_id                  # producing device

    # IPC path (preferred)
    bool   use_ipc
    string ipc_socket_path
    uint32 ipc_block_id
    uint64 ipc_uid                    # unique id for staleness detection

    # CPU fallback (empty when IPC is used)
    uint8[] serialized_data

The real CUDA backend
(``cuda_buffer_backend_msgs/CudaBufferDescriptor.msg``) illustrates the
device-IPC shape: the descriptor carries the publisher process id, block id,
socket path, and synchronization metadata needed to import a CUDA VMM block.
The exported file descriptor itself is passed out-of-band over the Unix-domain
socket named by the descriptor.
If the backend cannot serve a peer through CUDA IPC, it returns ``nullptr`` so
that peer receives the field through CPU fallback instead.

Step 2: Implement ``BufferImplBase<T>``
---------------------------------------

``BufferImplBase<T>`` is the pimpl that lives inside ``rosidl::Buffer<T>``.
Its interface is intentionally minimal:

.. code-block:: c++

    template<typename T>
    class BufferImplBase
    {
    public:
      virtual ~BufferImplBase() = default;
      virtual std::string get_backend_type() const = 0;
      virtual size_t size() const = 0;
      virtual std::unique_ptr<BufferImplBase<T>> to_cpu() const = 0;
      virtual std::unique_ptr<BufferImplBase<T>> clone() const = 0;
    };

* ``get_backend_type()`` must return the same short name your plugin's XML
  declares (``"cuda"``, ``"mydev"``, ...).
* ``size()`` returns the element count (not byte count, unless ``T`` is
  ``uint8_t``).
* ``to_cpu()`` is the escape hatch used by ``Buffer<T>::to_vector()`` when a
  consumer explicitly wants CPU memory; it must return a populated
  ``CpuBufferImpl<T>``.
* ``clone()`` produces a deep copy of ``*this``; it is used by
  ``Buffer<T>``'s copy constructor.

Beyond those four required methods, your implementation will usually hold
all of your backend's state: device pointers, IPC handles, cached streams,
staleness counters, and so on.
The plugin side of the interface then talks to this state through
``dynamic_cast`` to your concrete type.

Step 3: Implement ``BufferBackend``
-----------------------------------

.. _writing-buffer-backend_type-support:

Identity and descriptor type support
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``get_backend_type()`` returns the short name.
``get_descriptor_type_support()`` should return the **generic** aggregate
type support handle for your descriptor, not a specific typesupport
library's handle:

.. code-block:: c++

    const rosidl_message_type_support_t *
    MyDevBufferBackend::get_descriptor_type_support() const
    {
      return rosidl_typesupport_cpp::get_message_type_support_handle<
        mydev_buffer_backend_msgs::msg::MyDevBufferDescriptor>();
    }

This keeps the plugin RMW-agnostic.
Current RMW integrations resolve the aggregate to
``rosidl_typesupport_fastrtps_cpp`` at runtime, so any standard
``rosidl_default_generators`` message package will work out of the box.

``create_descriptor_with_endpoint`` / ``from_descriptor_with_endpoint``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

These are the two hot-path methods.

``create_descriptor_with_endpoint`` is called on the publisher side once per
matched remote endpoint.
It receives a type-erased non-owning pointer to a
``BufferImplBase<uint8_t>`` and the peer's ``rmw_topic_endpoint_info_t``.
Return a filled-in descriptor for that peer, or ``nullptr`` to tell the RMW
*"I cannot serve this peer with my backend; please fall back to CPU
serialization."*

Returning ``nullptr`` is a first-class signal.
Use it whenever:

* the peer does not support your backend (check
  ``endpoint_supported_backends`` during discovery and cache the answer);
* the hardware or environment cannot serve this pair (different device,
  different host, IPC disabled, ...);
* any other reason a non-CPU path is not available for *this* peer.

``from_descriptor_with_endpoint`` is the inverse, invoked on the subscriber
side.
It must produce a ``std::unique_ptr<void, void (*)(void *)>`` owning a
freshly-constructed ``BufferImplBase<T>``.
The custom deleter is **required** because the plugin may be unloaded
independently of the rest of the process; the deleter ensures destruction
happens inside the correct shared library.

A typical deleter looks like:

.. code-block:: c++

    auto impl = std::make_unique<MyDevBufferImpl<uint8_t>>(...);
    return {impl.release(), [](void * p) {
        delete static_cast<rosidl::BufferImplBase<uint8_t> *>(p);
      }};

Discovery hooks
^^^^^^^^^^^^^^^

``on_creating_endpoint`` lets your backend learn about local publishers and
subscriptions as they are created; this is usually where IPC managers
register themselves.

``on_discovering_endpoint`` is the compatibility negotiation step.
It returns a ``std::pair<bool, std::vector<std::set<uint32_t>>>``:

* The ``bool`` tells the RMW whether this peer is served by the backend at
  all.
  Returning ``false`` permanently routes this peer to CPU serialization.
* The ``std::vector<std::set<uint32_t>>`` is for backends that need to
  group endpoints (e.g. which subscribers share the same device and can use
  the same IPC handle).
  Most backends leave this empty.

The ``endpoint_supported_backends`` argument is a map from the peer's
advertised backend names to their metadata strings.
A correct backend **must** check that its own name is in this map before
declaring the peer compatible; otherwise a publisher will waste descriptor
work on a subscription that cannot consume it.

Simplified example from the CUDA backend:

.. code-block:: c++

    std::pair<bool, std::vector<std::set<uint32_t>>>
    CudaBufferBackend::on_discovering_endpoint(
      const rmw_topic_endpoint_info_t & /*endpoint_info*/,
      const std::vector<rmw_topic_endpoint_info_t> & /*existing_endpoints*/,
      const std::unordered_map<std::string, std::string> & endpoint_supported_backends)
    {
      if (endpoint_supported_backends.find("cuda") ==
          endpoint_supported_backends.end())
      {
        return {false, {}};
      }

      // The full implementation also checks VMM IPC support, endpoint
      // locality, device id, and user id before returning true.
      return {true, {}};
    }

Step 4: Register the plugin
---------------------------

Add a ``pluginlib`` XML file that declares your backend class:

.. code-block:: xml

    <!-- mydev_buffer_backend/mydev_buffer_plugin.xml -->
    <library path="mydev_buffer_backend">
      <class name="mydev"
             type="mydev_buffer_backend::MyDevBufferBackend"
             base_class_type="rosidl::BufferBackend">
        <description>Example backend for the mydev memory domain.</description>
      </class>
    </library>

The ``name`` attribute is the short name that users put into
``acceptable_buffer_backends`` (see
:doc:`Using-Buffer-Backends`) and that your
``get_backend_type()`` must return.

Export it from ``CMakeLists.txt``:

.. code-block:: cmake

    pluginlib_export_plugin_description_file(
      rosidl_buffer_backend mydev_buffer_plugin.xml)

and install it:

.. code-block:: cmake

    install(
      FILES mydev_buffer_plugin.xml
      DESTINATION share/${PROJECT_NAME}
    )

And register the class in your plugin source file:

.. code-block:: c++

    #include <pluginlib/class_list_macros.hpp>
    #include "mydev_buffer_backend/mydev_buffer_backend.hpp"

    PLUGINLIB_EXPORT_CLASS(
      mydev_buffer_backend::MyDevBufferBackend,
      rosidl::BufferBackend)

Step 5: Package and build system
--------------------------------

``package.xml`` for the plugin package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: xml

    <package format="3">
      <name>mydev_buffer_backend</name>
      <version>0.0.1</version>
      <description>mydev backend for rosidl::Buffer</description>
      <maintainer email="you@example.com">You</maintainer>
      <license>Apache License 2.0</license>

      <buildtool_depend>ament_cmake</buildtool_depend>

      <depend>rosidl_buffer_backend</depend>
      <depend>rosidl_buffer_backend_registry</depend>
      <depend>rosidl_runtime_cpp</depend>
      <depend>pluginlib</depend>
      <depend>mydev_buffer</depend>
      <depend>mydev_buffer_backend_msgs</depend>

      <export>
        <build_type>ament_cmake</build_type>
      </export>
    </package>

``CMakeLists.txt`` for the plugin package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The minimum shape is:

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.8)
    project(mydev_buffer_backend)

    find_package(ament_cmake REQUIRED)
    find_package(rosidl_buffer_backend REQUIRED)
    find_package(rosidl_buffer_backend_registry REQUIRED)
    find_package(rosidl_runtime_cpp REQUIRED)
    find_package(pluginlib REQUIRED)
    find_package(mydev_buffer REQUIRED)
    find_package(mydev_buffer_backend_msgs REQUIRED)

    add_library(${PROJECT_NAME} SHARED
      src/mydev_buffer_backend_plugin.cpp)

    target_include_directories(${PROJECT_NAME} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

    target_link_libraries(${PROJECT_NAME} PUBLIC
      rosidl_buffer_backend::rosidl_buffer_backend
      rosidl_buffer_backend_registry::rosidl_buffer_backend_registry
      rosidl_runtime_cpp::rosidl_runtime_cpp
      pluginlib::pluginlib
      mydev_buffer::mydev_buffer
      mydev_buffer_backend_msgs::mydev_buffer_backend_msgs__rosidl_typesupport_cpp)

    install(
      TARGETS ${PROJECT_NAME}
      EXPORT ${PROJECT_NAME}
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
      RUNTIME DESTINATION bin
      INCLUDES DESTINATION include/${PROJECT_NAME})

    install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
    install(FILES mydev_buffer_plugin.xml DESTINATION share/${PROJECT_NAME})

    pluginlib_export_plugin_description_file(
      rosidl_buffer_backend mydev_buffer_plugin.xml)

    ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
    ament_export_dependencies(
      rosidl_buffer_backend
      rosidl_buffer_backend_registry
      rosidl_runtime_cpp
      pluginlib
      mydev_buffer
      mydev_buffer_backend_msgs)

    ament_package()

See ``cuda_buffer_backend/CMakeLists.txt`` for a production-quality
version that also defines the test targets used by the reference launch
tests.

Step 6: Design a user-facing API
--------------------------------

Application authors do not typically construct ``BufferImplBase`` subclasses
by hand; they go through a user-facing API header shipped by the backend.
A minimal API usually provides three operations:

#. **Allocate a buffer** whose storage is already backed by your backend's
   memory:

   .. code-block:: c++

       rosidl::Buffer<uint8_t> allocate_buffer(std::size_t byte_count);

#. **Obtain a writable handle** to a backend-backed buffer so the caller can
   produce data in place (e.g. launch a kernel writing into the device
   pointer):

   .. code-block:: c++

       WriteHandle from_output_buffer(rosidl::Buffer<uint8_t> & buffer, ...);

#. **Obtain a readable handle** on the subscriber side, synchronised with
   whatever the publisher's write did (e.g. waiting on a shared event):

   .. code-block:: c++

       ReadHandle from_input_buffer(const rosidl::Buffer<uint8_t> & buffer, ...);

Typical auxiliary considerations:

* **Auto-promotion**: if a caller hands your backend a buffer that happens
  to live in some other backend (CPU, another device backend, ...), a
  helpful pattern is to allocate a new buffer in *your* domain and copy
  into it transparently, keeping ownership via a ``get_promoted_buffer()``
  getter on the returned handle.
  The CUDA backend does this for CPU-to-CUDA inputs.
* **Synchronisation**: if your backend is asynchronous (streams, events,
  queues), encode the synchronisation policy into your handle types rather
  than on the free functions.
  RAII on destruction makes correct use the default.

Higher-level libraries on top of backends
-----------------------------------------

Do not write a new backend just to add application-level metadata such as
tensor shape, point-cloud fields, or image color-space information.
Those semantics fit better in a normal ROS 2 message whose payload field is a
``uint8[]`` and therefore a ``rosidl::Buffer<uint8_t>`` in generated C++ code.
The message can then ride on whichever storage backend is negotiated for that
field.

The tensor packages in ``rosidl_buffer_backends`` follow this model:

* ``tensor_msgs/msg/ExperimentalTensor`` stores DLPack-aligned dtype, shape,
  stride, and byte-offset metadata plus a ``uint8[] data`` field.
* ``torch_conversions`` converts between that message and ``at::Tensor``.
  It allocates CUDA-backed ``data`` when requested and available, otherwise it
  uses CPU storage.
* Subscribers opt in to the underlying storage backend (for example
  ``acceptable_buffer_backends = "cuda"`` or ``"any"``), not to a separate
  tensor backend name.

This keeps the backend ecosystem focused on memory transport while allowing
ordinary user libraries to add richer programming models above it.
See :doc:`Writing-a-Buffer-Compatible-Conversions-Package` for a dedicated
guide to creating ``*_conversions`` packages on top of buffer-backed message
fields.

Testing
-------

The most useful tests for a backend are launch tests that exercise real
pub/sub topologies.
The reference CUDA backend ships a representative set:

* intra-process (publisher and subscriber in the same process);
* inter-process on the same host;
* mixed intra/inter-process (one publisher, multiple subscribers);
* multi-publisher (several producers into one topic);
* CPU fallback (subscriber that does not advertise the backend must still
  receive correct bytes);
* "mixed / no duplicate" (a subscription that accepts both the backend and
  CPU must not observe duplicated messages).

See ``cuda_buffer_backend/test/`` for the full set.

For finer-grained unit tests on the plugin interface itself (descriptor
round-trip without an RMW), the ``rosidl_buffer_backend_registry``
package's own tests and the ``test_rosidl_buffer`` system tests are good
starting points.

Ship checklist
--------------

Before releasing a backend, verify that:

* the plugin is discoverable, i.e. ``rosidl_buffer_backend_registry``'s
  ``get_backend_names()`` lists your short name;
* your descriptor serializes to ``<= 4096`` bytes for every shape you
  support;
* ``get_descriptor_type_support()`` returns the generic aggregate handle,
  not an RMW-specific one;
* ``create_descriptor_with_endpoint`` returns ``nullptr`` on every
  unsupported peer (cross-host, wrong device, IPC disabled, ...), and
  launch tests confirm CPU fallback works;
* ``from_descriptor_with_endpoint``'s returned ``unique_ptr`` uses a custom
  deleter bound inside your shared library;
* ``on_discovering_endpoint`` consults ``endpoint_supported_backends`` and
  refuses peers that do not advertise your backend;
* your package README includes a **transport-support matrix**
  (intra-process / inter-process same-host / inter-host / ...) so users
  know what to expect.

Where to go next
----------------

* :doc:`About-Buffer-Backends` -- conceptual
  overview of the feature.
* :doc:`Using-Buffer-Backends` -- the user-facing side
  of the same feature; useful for understanding what your plugin is
  expected to look like from a subscription's point of view.
* :doc:`GPU-Buffer-Transport` -- end-to-end demo of a
  GPU-backed tensor pub/sub pipeline using the CUDA backend and
  ``torch_conversions``.
