Glossary
========

.. include:: ../global_substitutions.txt

Glossary of terms used throughout this documentation:

.. glossary::

   API
       An API, or Application Programming Interface, is an interface that is provided by an "application", which in this case is usually a shared library or other language appropriate shared resource.
       APIs are made up of files that define a contract between the software using the interface and the software providing the interface.
       These files typically manifest as header files in C and C++ and as Python files in Python.
       In either case it is important that APIs are grouped and described in documentation and that they are declared as either public or private.
       Public interfaces are subject to change rules and changes to the public interfaces prompt a new version number of the software that provides them.

   client_library
       A client library is an :term:`API` that provides access to the ROS graph using primitive middleware concepts like Topics, Services, and Actions.

   package
       A single unit of software, including source code, build system files, documentation, tests, and other associated resources.

   REP
        Robotics Enhancement Proposal.
        A document that describes an enhancement, standardization, or convention for the ROS community.
        The associated REP approval process allows the community to iterate on a proposal until some consensus has been made, at which point it can be ratified and implemented, which then becomes documentation.
        All REPs are viewable from the `REP index <https://reps.openrobotics.org/>`_.

   VCS
       Version Control System, such as CVS, SVN, git, mercurial, etc...

   rclcpp
       The C++ specific :term:`Client Library <client_library>` for ROS.
       This includes any middleware related APIs as well as the related message generation of C++ data structures based on interface definitions like Messages, Services, and Actions.

   repository
       A collection of packages usually managed using a :term:`VCS` like git or mercurial and usually hosted on a site like GitHub or BitBucket.
       In the context of this document, repositories usually contain one or more |packages| of one type or another.

   Buffer
       ``rosidl::Buffer<T>``, the in-memory container used by generated C++ messages for variable-length primitive array fields (``uint8[]``, ``float32[]``, ...).
       It behaves like a ``std::vector<T>`` by default and supports pluggable memory backends so that vendors can back those fields with non-CPU memory.
       See :doc:`ROS-Framework/interfaces/Working-with-interfaces/Buffer-Backends/About-Buffer-Backends`.

   Buffer backend
       A ``pluginlib`` plugin, implementing the ``rosidl::BufferBackend`` interface, that teaches the RMW how to transport a ``rosidl::Buffer`` whose storage lives in a vendor-specific memory domain (for example, GPU memory).

   Tensor message
       A normal ROS 2 message, such as ``tensor_msgs/msg/ExperimentalTensor``, that carries tensor metadata and stores the raw tensor bytes in a ``uint8[]`` field backed by ``rosidl::Buffer``.
       Libraries such as ``torch_conversions`` can map these messages to framework-native tensor types while the underlying bytes use a regular buffer backend such as ``cpu`` or ``cuda``.

   Buffer descriptor
       A normal ROS 2 ``.msg`` produced by a ``BufferBackend`` that travels on the wire in place of the raw ``uint8[]`` contents of a buffer-backed field.
       Serialized descriptors must not exceed ``rosidl::kMaxBufferDescriptorSize`` (4096 bytes).

   Acceptable backend list
       The value of ``rclcpp::SubscriptionOptions::acceptable_buffer_backends`` (or ``acceptable_buffer_backends`` in ``rclpy``).
       A comma-separated list of backend names the subscription is willing to receive; ``"cpu"`` (or empty) means CPU-only, ``"any"`` means any installed backend, and CPU is always implicitly acceptable.
       See :doc:`ROS-Framework/interfaces/Working-with-interfaces/Buffer-Backends/Using-Buffer-Backends`.
