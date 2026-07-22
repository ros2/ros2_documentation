About ``rosidl::Buffer`` backends
=================================

.. toctree::
   :maxdepth: 1
   :hidden:

   Using-Buffer-Backends
   GPU-Buffer-Transport
   Writing-a-Buffer-Backend
   Writing-a-Buffer-Compatible-Conversions-Package


.. contents:: Table of Contents
   :local:

Overview
--------

``rosidl::Buffer<T>`` is a container type used as the in-memory representation
of variable-length primitive array fields in generated C++ messages
(``uint8[]``, ``float32[]``, etc.).
It is a drop-in replacement for ``std::vector<T>`` that also supports
pluggable memory backends, so the bytes of a ``uint8[]`` field can live in
CPU memory, GPU memory, or any other memory domain a vendor provides -- all
without changing the ``.msg`` file or the rest of the ROS 2 message pipeline.

The feature was introduced to let vendors transport large binary payloads
(camera images, point clouds, tensors, ...) through the existing ROS 2
pub/sub API with as few copies as the underlying memory technology allows,
while keeping every piece of existing code that treats a ``uint8[]`` field as
a ``std::vector<uint8_t>`` working unchanged.

Why a new type is needed
------------------------

ROS 2 messages already have two complementary mechanisms for reducing copies:

* :doc:`Intra-process communication <../../../nodes/Working-with-nodes/intra-process/Intra-Process-Communication>`
  avoids copies between publishers and subscriptions in the same process by
  passing ``std::unique_ptr`` ownership.
* :doc:`Loaned messages <../../../client-libraries/Working-with-Client-Libraries/Configure-ZeroCopy-loaned-messages>`
  let the RMW manage message memory to enable shared-memory transport between
  processes, for message types the underlying middleware can lay out.

Neither of these helps when the *source* of the data is a non-CPU memory
region, such as a GPU-rendered image or a tensor produced by an accelerator
runtime.
Copying such data into a ``std::vector<uint8_t>`` purely to satisfy the
generated C++ message type defeats the point of producing it on the
accelerator in the first place.

``rosidl::Buffer`` solves this at the container level: a message field that
is declared as ``uint8[]`` in the ``.msg`` file is still a byte array on the
wire and still behaves like a ``std::vector<uint8_t>`` by default, but its
implementation is a pluggable pimpl that backends can replace with
their own memory-domain-specific storage.

Architecture
------------

The feature is split across three core packages and a pluggable set of
vendor-provided backend packages.

Core packages
^^^^^^^^^^^^^

* ``rosidl_buffer`` -- defines the user-facing ``rosidl::Buffer<T>`` container
  and the ``rosidl::BufferImplBase<T>`` abstract base class that backends
  subclass.
  ``rosidl::Buffer<T>`` forwards every operation to the implementation it
  holds and provides implicit conversion to ``std::vector<T>&`` when the
  active backend is CPU, which is what preserves backward compatibility.
* ``rosidl_buffer_backend`` -- defines the ``rosidl::BufferBackend`` plugin
  interface that vendors implement.
  The interface is intentionally small: it advertises the backend's name and
  descriptor message type, creates and consumes descriptor messages on the
  publishing and receiving sides, and participates in endpoint discovery.
* ``rosidl_buffer_backend_registry`` -- discovers installed backend plugins
  via ``pluginlib`` and exposes them to the rest of the stack.

User-facing containers and implementation pimpls
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

   ┌──────────────────────────────┐
   │     rosidl::Buffer<T>        │  (what generated messages hold)
   │  ┌────────────────────────┐  │
   │  │ BufferImplBase<T>  ◄──-┼──┼── CpuBufferImpl<T>      (default)
   │  │ (pimpl)                │  │── CudaBufferImpl<T>     (vendor)
   │  └────────────────────────┘  │── OtherBufferImpl<T>    (vendor)
   └──────────────────────────────┘

A freshly-constructed ``rosidl::Buffer<T>`` uses ``CpuBufferImpl<T>``, which
simply wraps a ``std::vector<T, Allocator>``.
A backend swaps this out for its own ``BufferImplBase<T>`` subclass when it
allocates a buffer (on the publisher side) or reconstructs one from a
received descriptor (on the subscriber side).

The backend plugin
^^^^^^^^^^^^^^^^^^

The ``rosidl::BufferBackend`` plugin is what the RMW layer talks to.
Its job is essentially to translate between an in-memory
``BufferImplBase<T>`` and a **descriptor message** -- a normal ROS 2 ``.msg``
that describes how to locate or reconstruct the payload on the receiving
side.
For a CPU-only backend the descriptor can carry the bytes directly.
For a non-CPU backend the descriptor is usually a small reference that the receiving side uses to re-attach to the payload; the exact mechanism is backend-specific.

Descriptor messages are bounded (``rosidl::kMaxBufferDescriptorSize``, 4096
bytes) so that the RMW can plan serialization buffer sizes up front.

Descriptor round-trip and RMW integration
-----------------------------------------

When a publisher sends a message that contains a ``rosidl::Buffer<T>`` field
with a non-CPU backend, the RMW:

#. asks the backend to build a descriptor for the current peer
   (``create_descriptor_with_endpoint``);
#. serializes the descriptor in place of the raw ``uint8[]`` bytes;
#. on the subscriber side, deserializes the descriptor and asks the backend
   to rebuild a ``BufferImplBase<T>`` from it
   (``from_descriptor_with_endpoint``).

Discovery hooks
^^^^^^^^^^^^^^^

Backends are told about every matched endpoint via
``on_creating_endpoint`` (local) and ``on_discovering_endpoint`` (remote).
They use these hooks to decide whether a given pub/sub pair is actually compatible with the backend's transport.
For instance, the CUDA backend currently accepts peers only when it can share CUDA VMM allocations safely.
If a backend cannot serve a particular peer, it returns ``nullptr`` from
``create_descriptor_with_endpoint`` and the RMW falls back to normal CPU
serialization of the field.

RMW-agnostic plugin contract
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The plugin interface itself is RMW-agnostic.
``BufferBackend::get_descriptor_type_support()`` returns the generic
``rosidl_message_type_support_t *`` aggregate handle (the same handle
``rosidl_typesupport_cpp::get_message_type_support_handle<T>()`` produces)
for the backend's descriptor message type.
The consuming RMW resolves that aggregate to the concrete per-typesupport
library it needs at runtime.
The current ``rmw_fastrtps_cpp`` integration resolves this aggregate to
``rosidl_typesupport_fastrtps_cpp``, but nothing in the backend API ties
it to a specific RMW.

Backends and higher-level libraries
-----------------------------------

A backend should represent a memory substrate or transport technology.
Examples include a CUDA backend built on CUDA VMM and CUDA IPC, a ROCm
backend, or a shared-memory backend.
The backend knows how to allocate memory, package that memory into a
descriptor, and re-import it on another endpoint.

Higher-level data models can live above this layer as ordinary messages and
libraries.
For example, ``tensor_msgs/msg/ExperimentalTensor`` carries DLPack-aligned
tensor metadata (dtype, shape, strides, byte offset) plus a ``uint8[] data``
field.
That ``data`` field is a ``rosidl::Buffer<uint8_t>`` in generated C++ code, so
it can be transported by whichever backend is negotiated for the connection.
The ``torch_conversions`` helper library converts between
``ExperimentalTensor`` and ``at::Tensor``; it is not itself a buffer backend.
When the message's data buffer uses the ``cuda`` backend, tensor bytes can
move through CUDA IPC.
When it uses the CPU backend, the same message and helper APIs still work with
ordinary host memory.

Relationship to other ROS 2 mechanisms
--------------------------------------

* ``rosidl::Buffer`` is orthogonal to :doc:`intra-process communication
  </ROS-Framework/nodes/Working-with-nodes/intra-process/Intra-Process-Communication>` and to
  :doc:`loaned messages <../../../client-libraries/Working-with-Client-Libraries/Configure-ZeroCopy-loaned-messages>`.
  A backend may implement either, both, or neither for a given pub/sub pair;
  the decision lives entirely inside the backend.
* Whether a given publisher/subscriber pair can actually use a non-CPU
  transport (intra-process, inter-process same-host, inter-host, ...) is a
  property of the backend implementation, not of ``rosidl::Buffer``.
  Consult each backend's own documentation for its support matrix.
* The ``.msg`` IDL does not change: a field that was ``uint8[]`` before is
  still ``uint8[]``; only its generated C++ type changed from
  ``std::vector<uint8_t>`` to ``rosidl::Buffer<uint8_t>``, and implicit
  conversion keeps most existing code working.
* The current RMW integration applies to topic publish/subscribe.
  Services and actions continue to use their normal serialization paths and do not negotiate non-CPU buffer backends.

Relationship to type adaptation
------------------------------------------

``rosidl::Buffer`` backends operate at the generated-message container layer.
The ROS message definition remains the topic type, while selected variable-length primitive array fields can use backend-specific storage and descriptor-based transport.

Type adaptation and systems such as Isaac ROS NITROS solve a different problem: they let application code work with framework-native types and negotiate adapted representations above the ROS message type, using mechanisms such as `REP 2007 <https://reps.openrobotics.org/rep-2007/>`__ and `REP 2009 <https://reps.openrobotics.org/rep-2009/>`__.
The two approaches are not part of the same abstraction and are not intended to depend on each other; their scopes are different.
They can coexist in an application when both are useful.
For example, an adapted application type can still contain or produce a ROS message whose ``uint8[]`` field is backed by ``rosidl::Buffer``.

One practical difference is that buffer backends are visible to the RMW publish/subscribe path, so a backend can provide cross-process transport support while type adaptation can only work within a single process.

Where to go next
----------------

* :doc:`Using-Buffer-Backends` -- user-facing guide on
  enabling a buffer backend for a subscription and reading/writing the
  backend-native data.
* :doc:`Writing-a-Buffer-Backend` -- vendor-facing
  guide on implementing and packaging a new ``BufferBackend`` plugin.
* :doc:`Writing-a-Buffer-Compatible-Conversions-Package` --
  guide on creating ``*_conversions`` packages for messages with
  buffer-backed ``uint8[]`` fields.
* :doc:`GPU-Buffer-Transport` -- end-to-end demo that
  exercises a GPU-backed publish/subscribe pipeline.
