Creating an ``rmw`` implementation
==================================

**Goal:** Learn how to create a new ``rmw`` implementation, from the features required from the underlying middleware to the ``rmw`` implementation details.

**Tutorial level:** Advanced

**Time:** 30+ minutes

.. contents:: Table of Contents
   :local:

Introduction
------------

ROS 2's architecture has two main :doc:`abstraction layers <../../Concepts/Advanced/About-Internal-Interfaces>`.
From top to bottom:

#. The client library interface, ``rcl``, which supports the user-facing :doc:`client libraries <../../Concepts/Basic/About-Client-Libraries>`, such as ``rclcpp`` and ``rclpy``
#. The middleware interface, ``rmw``, which abstracts away the :doc:`underlying middleware implementation <../../Concepts/Intermediate/About-Different-Middleware-Vendors>`, such as a specific DDS implementation, Zenoh, etc.

The ``rmw`` `API includes function-level documentation <https://docs.ros.org/en/{DISTRO}/p/rmw/generated/index.html#functions>`_, but there is no higher-level documentation on the features of the interface and what it expects from the underlying middleware.

This guide is for developers who want to implement the ``rmw`` interface for a specific middleware.
It will first go over the ``rmw`` interface and how it works.
Then it will cover the main concepts or features that a middleware implementation must support.
Finally, it will go over some implementation details, including how to create an implementation skeleton and some tips to implement the interface functions.

This guide is intended to be an entry point to kickstart the development of a new ``rmw`` implementation.
It will link to other pages and source code for more details where appropriate.

.. note::

    ROS 2 design articles on `design.ros2.org <https://design.ros2.org/>`_ are historical documents and may not reflect the current state of ROS 2.
    However, in some cases, they provide useful context and information, so they may still be referenced by this guide or by pages that this guide links to.

The ``rmw`` interface
---------------------

The ``rmw`` interface is declared by the ``rmw`` package through `C header files <https://github.com/ros2/rmw/tree/{DISTRO}/rmw/include/rmw>`_.
Implementations of the C functions declared in these headers are provided by ``rmw`` implementations, which are separate packages.
For example, the ``rmw_fastrtps_cpp`` package implements the interface for eProsima Fast DDS.

Example implementations
^^^^^^^^^^^^^^^^^^^^^^^

The following ``rmw`` :doc:`implementations <../../Concepts/Advanced/About-Middleware-Implementations>` can be used as references.
Note that there are different `support tiers, which are defined by REP 2000 <https://www.ros.org/reps/rep-2000.html#support-tiers>`_.

#. DDS:

    #. ``rmw_fastrtps_cpp``, ``rmw_fastrtps_dynamic_cpp``: `ros2/rmw_fastrtps <https://github.com/ros2/rmw_fastrtps>`_
    #. ``rmw_cyclonedds_cpp``: `ros2/rmw_cyclonedds <https://github.com/ros2/rmw_cyclonedds>`_
    #. ``rmw_connextdds``: `ros2/rmw_connextdds <https://github.com/ros2/rmw_connextdds>`_
    #. ``rmw_gurumdds_cpp``: `ros2/rmw_gurumdds <https://github.com/ros2/rmw_gurumdds>`_

    * See :ref:`this overview <about-middleware-impls_struct_dds>`

#. ``rmw_zenoh_cpp``: `ros2/rmw_zenoh <https://github.com/ros2/rmw_zenoh>`_

    * See :ref:`this overview <about-middleware-impls_struct_zenoh>` and the `design document <https://github.com/ros2/rmw_zenoh/blob/{DISTRO}/docs/design.md>`_

#. ``rmw_email_cpp``, an email-based implementation: `christophebedard/rmw_email <https://github.com/christophebedard/rmw_email>`_

    * See the `design document for the underlying email middleware <https://christophebedard.com/rmw_email/design/email/>`_ and the `blog post for some context <https://christophebedard.com/ros-2-over-email/>`_

.. _rmw-impl-guide_selection-mechanism:

Build-time and runtime ``rmw`` implementation selection mechanism
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The dependency on the actual ``rmw`` implementation is done through the ``rmw_implementation`` `package <https://index.ros.org/p/rmw_implementation/#{DISTRO}>`_.
Users of ``rmw``, such as ``rcl``, depend on the ``rmw`` package for the interface (headers) and some utility functions.
They also depend on the ``rmw_implementation`` package to get the actual implementation.

By default, ROS 2 allows you to choose which ``rmw`` implementation to use at runtime.
This is convenient for comparing two implementations on the same machine, and it lets ROS 2 distribute a single set of binaries that is compatible with multiple ``rmw`` implementations.
The :doc:`implementation is selected at runtime <../../How-To-Guides/Working-with-multiple-RMW-implementations>` through the ``RMW_IMPLEMENTATION`` environment variable, or, if that variable is unset, a default ``rmw`` implementation is loaded.

This is accomplished by the ``rmw_implementation`` package, which acts as a proxy for an actual ``rmw`` implementation.
It works by creating placeholder ``rmw`` functions.
When they are called, it will ``dlopen()`` the appropriate library for the selected ``rmw`` implementation and then look up the corresponding symbols in the loaded shared library using the ``dlsym()`` function before calling them.

The ``rmw_implementation`` package can be configured at build-time to change the default option or disable runtime selection.
The default implementation can be selected at build-time with the ``RMW_IMPLEMENTATION`` CMake variable (e.g., ``-DRMW_IMPLEMENTATION=rmw_other``) or the ``RMW_IMPLEMENTATION`` environment variable.
If only one implementation is available at build-time, or if runtime selection is disabled (``-DRMW_IMPLEMENTATION_DISABLE_RUNTIME_SELECTION=ON``), the ``rmw_implementation`` target will be a simple ``INTERFACE`` library for the single implementation.

Because of the proxy mechanism and the CMake logic described above, an ``rmw`` implementation that does not implement all of the functions in the interface will only fail at runtime, when the symbol lookup fails, as opposed to failing at build-time (specifically at link-time) if runtime selection is disabled.

Features
--------

This section goes over the main features of the ``rmw`` interface, which the underlying middleware must support or deal with.
Depending on the middleware -- and how similar it is to the features expected by the interface -- the ``rmw`` implementation may be more or less trivial, i.e., it might have to do more "glue" work.
For some non-critical features or configuration options, the implementation can indicate that they are not supported through ``rmw_feature_supported()`` or by returning ``RMW_RET_UNSUPPORTED``.
In any case, any special behavior of the ``rmw`` implementation should ideally be documented.

Topics, pub/sub, services
^^^^^^^^^^^^^^^^^^^^^^^^^

:doc:`Topics <../../Concepts/Basic/About-Topics>` are a common concept in publish/subscribe middleware.
However, ROS 2 has its own topic name conventions, which is validated using ``rmw_validate_full_topic_name()``.
The ``rmw`` implementation simply has to use the given (resolved) topic name.
This might involve adapting or mangling the ROS topic name to fit the underlying middleware's topic name conventions or constraints, or encode useful information.
For example, a pub/sub topic called ``/chatter`` is usually mangled into ``rt/chatter`` for DDS-based implementations, making ROS topics on DDS easily distinguishable from normal DDS topics.
See the `"Mapping of ROS 2 Topic and Service Names to DDS Concepts" section in this design document <https://design.ros2.org/articles/topic_and_service_names.html#mapping-of-ros-2-topic-and-service-names-to-dds-concepts>`_.
For Zenoh, the domain ID, resolved topic name, topic type name, and topic type hash are `encoded in the underlying Zenoh key <https://github.com/ros2/rmw_zenoh/blob/{DISTRO}/docs/design.md#topic-and-service-name-mapping-to-zenoh-key-expressions>`_ to avoid communications between different ROS topic names & types.

As for :doc:`services <../../Concepts/Basic/About-Services>`, they are not always natively supported by the underlying middleware.
For DDS-based implementations, they are simply built on top of pub/sub: 1 request topic and 1 response topic.
[#fn_dds_rpc]_
On the other hand, Zenoh natively supports services through `queryables <https://github.com/ros2/rmw_zenoh/blob/{DISTRO}/docs/design.md#service-servers>`_, so they are used to implement services in ``rmw_zenoh_cpp``.

Note that, while services are a part of the ``rmw`` interface, :doc:`actions <../../Concepts/Basic/About-Actions>` are not.
They are an ``rcl`` concept implemented in the ``rcl_action`` package on top of services and pub/sub.

Nodes
^^^^^

:doc:`Nodes <../../Concepts/Basic/About-Nodes>` are mostly a ROS concept.
Neither DDS nor Zenoh has a corresponding concept, so they are mostly a logical concept in the ``rmw`` implementation.
Topic names get resolved with the node namespace/name, if needed, by ``rcl`` before they are passed to ``rmw`` when creating a pub/sub object.
Implementations just have to make sure to include nodes in :ref:`introspection data <rmw-impl-guide_introspection>`.

.. _rmw-impl-guide_waitsets:

Wait sets and waiting
^^^^^^^^^^^^^^^^^^^^^

:doc:`Executors <../../Concepts/Intermediate/About-Executors>` are responsible for triggering user-provided callbacks when a new message is received, for example.
Executors are implemented at the client library level (``rclcpp``, ``rclpy``), but they rely on the underlying middleware to wait for new messages using a polling mechanism.
This is done using wait sets, which allow waiting on different entities at the same time in a standard way, e.g., subscriptions, service clients, and service servers.
The ``rmw_wait()`` `function <https://docs.ros.org/en/{DISTRO}/p/rmw/generated/function_rmw_8h_1a5f480dd59075e80288fb596b2951be2b.html>`_ is called with lists of entities to wait on, as well as an implementation-specific wait set object.
It adds all entities to the wait set and asks it to wait until at least one entity has new data available or until it times out.
Then the executor checks the lists of entities to see which ones have new data available and triggers the corresponding callbacks.

The key mechanism here is the ability to check if a given entity is ready, e.g., check if a subscription has a new message.
Then waiting simply involves continuously checking entities one at a time until one is ready or until the wait times out.

Take a look at how ``rmw_email_cpp`` `implements wait sets and waiting <https://github.com/christophebedard/rmw_email/blob/72742241d55f306d1dddcaf5dd6a5d6c2d402433/rmw_email_cpp/src/rmw_wait.cpp#L133>`_ and dig down to the middleware, ``email``, since it's fairly simple.

Taking data
^^^^^^^^^^^

Once an :ref:`executor is done waiting <rmw-impl-guide_waitsets>` and there is a new message, request, or response, it takes it from the middleware and triggers the corresponding callback.
For instance, ``rmw_take()`` is called with a subscription and a type-erased pointer to an instance of the corresponding message type to write to.

``rmw_email_cpp`` takes the new message (YAML string) from the underlying email middleware subscription object, and converts it into a ROS message by writing into the provided message.

Metadata: GIDs, timestamps, sequence numbers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Aside from actual user-specified data, message publications, service requests, service responses, and so on also have metadata associated with them:

* GID: globally-unique ID that identifies an entity (e.g., pub, sub, client, server)

    * The GID for an entity should be unique within a ROS domain and should be the same when reported both locally and remotely.
      For example, the publisher GID for a message being published should be the same publisher GID reported on the other side, when that message is received by a subscription.
      [#fn_gid_remote_matching]_

* Source & received timestamps: publication & subscription reception timestamps, respectively

* Publication & reception sequence numbers

This means that service request metadata includes the GID of the client that made the request and the request sequence number.
Service response metadata also includes the client GID & sequence number of the request it is responding to.

This metadata is available as structs through ``rmw_take_with_info()`` for subscription messages and ``rmw_take_{request,response}()`` for service requests/responses, which are wrapped & provided to user callbacks by the client libraries.

Part of this metadata might be natively supported and provided by the underlying middleware, while another part might have to be included and transmitted alongside the application data by the ``rmw`` implementation.
For instance, DDS natively supports all of it for pub/sub through DDS sample info, but the client request metadata needs to be wrapped alongside the service response data by the ``rmw`` implementation.
``email`` natively supports all of this metadata, which is included in standard email headers (i.e., not in the email body).

.. _rmw-impl-guide_typesupport:

Type support
^^^^^^^^^^^^

To bridge the gap between ROS 2 :doc:`interfaces <../../Concepts/Basic/About-Interfaces>` (specifically :doc:`custom interfaces <../Beginner-Client-Libraries/Custom-ROS2-Interfaces>`) and the underlying middleware, some glue code is needed.
This is referred to as :ref:`type support <Type Specific Interfaces>`.
When publishing a message of type {interface(std_msgs/msg/String)}, ``rmw_publish()`` only gets a ``void *`` to the message, which could point to a C++ instance, or a C instance, and so on.
The pointer will be interpreted based on the type support information provided when the publisher was created.

First, code is generated for each combination of interface type and user-facing language, independent of the underlying middleware.
For example, for the {interface(std_msgs/msg/String)} message type, data structures are generated:

#. C++: ``std_msgs/msg/string.hpp`` header with ``std_msgs::msg::String`` class generated by the ``rosidl_generator_cpp`` package
#. C: ``std_msgs/msg/string.h`` header with ``std_msgs__msg__String`` struct generated by the ``rosidl_generator_c`` package
#. Python: ``std_msgs`` module with ``std_msgs.msg.String`` class (which is just a wrapper around the C struct) generated by the ``rosidl_generator_py`` package
#. (and so on, e.g., for Rust)

Second, for the underlying middleware to be able to send and receive messages, it needs to know how to interpret the user-facing data structure.
This is one of the most critical parts of an ``rmw`` implementation.
There are two options: :ref:`static type support <internal-interfaces_static-type-support>` and :ref:`dynamic type support <internal-interfaces_dynamic-type-support>`.
Static type support involves generating middleware-specific code for each interface.
For example, ``rosidl_typesupport_fastrtps_cpp`` generates code to serialize/deserialize C++ classes of each interface type into CDR using `Fast CDR <https://github.com/eProsima/Fast-CDR>`_ for ``rmw_fastrtps_cpp`` to pass on to Fast DDS.
[#fn_ts_fastrtps]_
``rmw_connextdds`` and even ``rmw_zenoh_cpp`` use CDR for serialization, so they use this type support package as well.
On the other hand, dynamic type support involves generating a bit of middleware-independent code that provides generic information about each interface type.
[#fn_ts_dynamic]_

This information can be used at runtime by any ``rmw`` implementation to interpret a type-erased pointer to data: names & types of fields, functions to read from/write to fields depending on their type, functions to get the size of an array field, etc.
For C++, this is ``rosidl_typesupport_introspection_cpp``, which is used by ``rmw_fastrtps_dynamic_cpp`` (hence the "dynamic" part), for example.

Dynamic type support is generally slower than static type support at runtime because it has to iterate over each message field, figure out what type it is, and then process it, e.g., serialize it.
Static type support knows exactly how to process the message thanks to the code it generated for each interface type.
This is why most ``rmw`` implementations use static type support.
However, dynamic type support does not require generating middleware-specific code.
Choosing between static and dynamic type support is an orthogonal decision to the ``rmw`` implementation itself.

``rmw_email_cpp`` uses dynamic type support to convert messages to and from YAML string to be sent over email.
It gets the type support introspection information, and passes it and the message to an external/experimental package, `dynmsg <https://github.com/osrf/dynamic_message_introspection/>`_, which converts the message to/from YAML.
The YAML object is then sent as a YAML-formatted string via email using the underlying middleware.
When a new message is received by the middleware, the YAML string is converted into a message.

Domain ID
^^^^^^^^^

:doc:`Domain IDs <../../Concepts/Intermediate/About-Domain-ID>` are a way to have separate logical networks on the same physical network.
It is a native feature of DDS, but not Zenoh.
DDS achieves this by using the domain ID as a network port offset, while Zenoh implements it by making the domain ID the first component of the internal Zenoh key corresponding to each ROS 2 topic.

Quality of service (QoS)
^^^^^^^^^^^^^^^^^^^^^^^^

:doc:`Quality of service settings <../../Concepts/Intermediate/About-Quality-of-Service-Settings>` in ROS 2 are largely derived from DDS.
Basic QoS policies like history, depth, and durability are the same as ROS 1's, but more advanced policies simply come from DDS.
Implementations may simply ignore some settings.
For instance, ``rmw_zenoh_cpp`` doesn't implement the deadline and lifespan QoS policies.

One important aspect of QoS is that two profiles, e.g., a publisher's profile and a subscription's profile, may be incompatible, meaning they cannot communicate.
It is up to the implementation to decide if two QoS profiles are compatible: ``rmw_qos_profile_check_compatible()``.
DDS-based implementations rely on ``rmw_dds_common::qos_profile_check_compatible()``, since :ref:`QoS profile compatibility <about-qos_compatibilities>` is standard in DDS.
In Zenoh, `QoS settings are essentially never incompatible <https://github.com/ros2/rmw_zenoh/blob/{DISTRO}/docs/design.md#quality-of-service>`_.

To support a universal "default" behavior, QoS policies include a ``*_SYSTEM_DEFAULT`` setting (e.g., ``rmw_qos_reliability_policy_t``'s ``RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT``), which leaves the value up to the middleware implementation.
Then the ``rmw_*_get_actual_qos()`` functions retrieve the actual QoS profile used by the implementation.

.. _rmw-impl-guide_introspection:

ROS graph introspection
^^^^^^^^^^^^^^^^^^^^^^^

Nodes are able to get a list of other nodes, topics, etc.
This also allows publishers to know if any subscriptions exist for their topic, for example.
This same mechanism is used to :doc:`list nodes <../Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes>`, :doc:`topics <../Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics>`, and so on with the ROS 2 CLI: ``ros2 node list``, ``ros2 topic list``, etc.

This is supported by a number of ``rmw`` functions: ``rmw_get_node_names()``, ``rmw_get_topic_names_and_types()``, ``rmw_publisher_count_matched_subscriptions()``, and many more.
While the implementation is not specified by the interface, ``rmw`` implementations usually maintain a cache of the ROS graph.
When they create a new entity (e.g., node, publisher, subscription, service, client), they note it in their internal graph cache and notify other participants through a middleware-specific mechanism so that they can add it to their cache.
The graph cache belongs to the ``rmw`` context, so it is initialized when ``rmw_init()`` is called.
This context indirectly belongs to the ``rclcpp`` context (e.g., initialized by ``rclcpp::init()``), so there is usually only one graph cache per process.

Since DDS-based ``rmw`` implementations are very similar in this regard, they share a common graph cache implementation in the ``rmw_dds_common`` `package <https://github.com/ros2/rmw_dds_common>`__.
It uses an internal topic (usually ``ros_discovery_info``) to share information about new entities.
``rmw_zenoh_cpp`` `creates a Zenoh liveliness token <https://github.com/ros2/rmw_zenoh/blob/{DISTRO}/docs/design.md#graph-cache>`_ with the entity type & info and shares it with other participants.

Events
^^^^^^

Users can provide callbacks for publishers & subscriptions to be triggered by the middleware (but executed by the client library) on certain events (``rmw_event_type_t``), such as :ref:`quality of service-related events <about-qos_qos-events>` and :ref:`pub-sub match events <about-qos_matched-events>`.
Some of these events could be triggered on relevant changes to the graph cache.

Security
^^^^^^^^

:doc:`Security <../../Concepts/Intermediate/About-Security>` is not well-specified by the ``rmw`` interface; most of it is specified by :doc:`SROS2 <../Advanced/Security/Introducing-ros2-security>`.
The interface only defines a few security options as part of the context initialization options, ``rmw_init_options_t``:

#. ``rmw_security_options_t``, which includes a security policy (enforce/permissive) and a path to a directory containing security artifacts, i.e., a keystore.
   These are set by ``rcl`` based on environment variables: ``ROS_SECURITY_ENABLE`` & ``ROS_SECURITY_STRATEGY`` and ``ROS_SECURITY_KEYSTORE``.
#. The name of a security enclave from the keystore to use for the given process.
   This is set, for example, through the ``--enclave`` option when running a node with ``ros2 run``.

However, in practice, the structure of the :doc:`keystore <./Security/The-Keystore>` directory and its security enclaves is based on the DDS Security specification.
Therefore, :doc:`security artifacts generated <./Security/Introducing-ros2-security>` with the ``sros2`` package can only be directly used by DDS-based ``rmw`` implementations.
For ``rmw_zenoh_cpp``, `Zenoh-specific security configuration files can be generated <https://github.com/ros2/rmw_zenoh/tree/{DISTRO}/zenoh_security_tools>`_ from ``sros2``-generated artifacts using the ``zenoh_security_tools`` package and provided through the ``ZENOH_SESSION_CONFIG_URI`` environment variable, bypassing the ``ROS_SECURITY_*`` environment variables.

Implementation
--------------

Implementation skeleton
^^^^^^^^^^^^^^^^^^^^^^^

This section covers concrete steps to create the base files and directories for the new implementation package, including special handling in ``package.xml`` and ``CMakeLists.txt``.

Start with the :doc:`package creation tutorial <../../Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package>` to create an empty package.
Then make the following changes:

#. ``package.xml``

    #. Define a package/implementation name

        The package name is also the name of the ``rmw`` implementation.
        It will be used to :ref:`select the implementation <rmw-impl-guide_selection-mechanism>` through the ``RMW_IMPLEMENTATION`` environment variable or CMake option, for example.
        The name usually starts with ``rmw_`` and is followed by the name of the underlying middleware.
        Most :doc:`implementations in the ROS 2 ecosystem <../../Concepts/Intermediate/About-Different-Middleware-Vendors>` then append a suffix such as ``_cpp`` to indicate that the implementation is written in C++.
        However, that is not required.
        Examples: ``rmw_fastrtps_cpp``, ``rmw_cyclonedds_cpp``, ``rmw_connextdds``, ``rmw_zenoh_cpp``, and ``rmw_email_cpp``.

        .. code-block:: xml

            <!-- TODO replace with the actual implementation name -->
            <name>rmw_IMPLEMENTATION_NAME_cpp</name>

    #. Declare a dependency on ``rmw``

        Since the package will implement the interface declared in the ``rmw`` package and will depend on some utility functions.

        .. code-block:: xml

            <depend>rmw</depend>

    #. Declare a dependency on the required type support packages

        See the :ref:`type support section <rmw-impl-guide_typesupport>` for details.

        .. code-block:: xml

            <!-- keep or add what is necessary -->
            <depend>rosidl_typesupport_fastrtps_c</depend>
            <depend>rosidl_typesupport_fastrtps_cpp</depend>
            <depend>rosidl_typesupport_introspection_c</depend>
            <depend>rosidl_typesupport_introspection_cpp</depend>

    #. Declare membership of the ``rmw_implementation_packages`` group

        This allows the ``rmw_implementation`` package to `depend on the implementation <https://github.com/ros2/rmw_implementation/blob/4dd5d571a5bfa1a67183acf271dfa442932c7572/rmw_implementation/package.xml#L38>`_ so that it gets built alongside other implementations, since no package otherwise explicitly depends on any ``rmw`` implementations.
        This way it can be found and used if selected.

        .. code-block:: xml

            <member_of_group>rmw_implementation_packages</member_of_group>

#. ``CMakeLists.txt``

    #. Create the library target

        The library has to be a shared library.
        It should depend on ``rmw`` for headers and utility functions as well as the required type support packages.
        It will also depend on the underlying middleware.

        .. code-block:: cmake

            add_library(${PROJECT_NAME} SHARED
              src/file.cpp
              # ...
            )
            target_link_libraries(${PROJECT_NAME} PUBLIC
              rmw::rmw
            )
            target_link_libraries(${PROJECT_NAME} PRIVATE
              rosidl_typesupport_fastrtps_c::rosidl_typesupport_fastrtps_c
              rosidl_typesupport_fastrtps_cpp::rosidl_typesupport_fastrtps_cpp
              rosidl_typesupport_introspection_c::rosidl_typesupport_introspection_c
              rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp
              # TODO add any implementation-specific dependencies, e.g., underlying middleware
            )

    #. Configure the implementation library target

        In practice, this simply makes symbols hidden by default to hide internal symbols, i.e., non-``rmw`` interface symbols.
        If the implementation is written in C (not common), specify ``LANGUAGE "C"``.

        .. code-block:: cmake

            configure_rmw_library(${PROJECT_NAME})

    #. Register the ``rmw`` implementation

        This registers the implementation in the :ref:`ament index <ament-cmake-doc_adding-resources>` so that it can be found at build-time (``get_available_rmw_implementations()``, ``get_rmw_typesupport()``) or at runtime (``ament_index_cpp::get_resources("rmw_typesupport")``).
        It also registers the languages that the implementation supports and the list of type support packages.
        For example, if the implementation only uses type support introspection (i.e., dynamic and not static) for C and C++ messages:

        .. code-block:: cmake

            register_rmw_implementation(
              "c:rosidl_typesupport_introspection_c"
              "cpp:rosidl_typesupport_introspection_cpp"
            )

        .. For example, the Python client library, ``rclpy``, uses C typesupport, and checks if C typesupport is available using ``get_rmw_typesupport()``, so the ``rmw`` implementation must support it.

    #. Install and export target

        .. code-block:: cmake

            install(
              TARGETS ${PROJECT_NAME}
              EXPORT ${PROJECT_NAME}
              ARCHIVE DESTINATION lib
              LIBRARY DESTINATION lib
              RUNTIME DESTINATION bin
            )

            ament_export_targets(${PROJECT_NAME})
            # ament_export_libraries(${PROJECT_NAME})  # Old-style CMake

            # ...

Interface functions implementation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The first step is to define the C interface functions declared in the ``rmw`` headers.
Start with empty functions that simply return ``RMW_RET_OK``, and then implement them one by one in the order in which they are likely to be called at runtime.
For example: ``rmw_init()``, ``rmw_create_node()``, ``rmw_create_publisher()``, ``rmw_create_subscription()``, and so on.
This will allow building and running/testing the implementation incrementally.

Most ``rmw`` functions have to perform input validation, as defined by the function's documentation.
There are various utility macros to simplify this, such as ``RMW_CHECK_ARGUMENT_FOR_NULL()`` and ``RMW_CHECK_TYPE_IDENTIFIERS_MATCH()``.

``rmw`` structs usually include a type-erased pointer (or sometimes an opaque pointer) for ``rmw`` implementation-specific data.
For instance, ``rmw_publisher_t`` has a ``void * data``.
The implementation can place whatever it wants there, e.g., a pointer to an internal object that wraps the underlying middleware's publisher object and any relevant information, like type support.
This data/object can be fetched and used later when ``rmw_publish()`` is called with the corresponding ``rmw_publisher_t``.
To make sure that a different ``rmw`` implementation doesn't try to interpret this data, ``rmw_publisher_t`` includes the name of the implementation in its ``implementation_identifier`` field.

Type support
^^^^^^^^^^^^

Type support structs can be confusing.
Here is an example for message type support for publishers/subscriptions.

A publisher is created through ``rmw_create_publisher()``, which takes in a handle for the type support information: ``const rosidl_message_type_support_t *``.
This is the base language-dependent type support: ``rosidl_typesupport_c`` / ``rosidl_typesupport_cpp``.
From this, we can get the concrete type support handle, depending on the available type supports, e.g., ``rosidl_typesupport_fastrtps_c`` / ``rosidl_typesupport_fastrtps_cpp`` and ``rosidl_typesupport_introspection_c`` / ``rosidl_typesupport_introspection_cpp``.
The confusing part is that these are also of type ``const rosidl_message_type_support_t *``!
However, the concrete type support handles are the ones that contain actual useful information.
See `this example function <https://github.com/christophebedard/rmw_email/blob/f5e622bab24edaad8e0da054c7dbc698c6fb809c/rmw_email_cpp/src/type_support.cpp#L29-L62>`__, which extracts the concrete C or C++ dynamic message type support handle (``rosidl_typesupport_introspection_{c,cpp}``) given a base type support handle (``rosidl_typesupport_{c,cpp}``).
Publishers created by ``rclcpp`` will use C++ type support, while publishers created by ``rclpy`` will use C type support, since Python messages get converted into C messages.
The ``/rosout`` publisher is managed by ``rcl``, which is written in C, so it uses C type support.

Then, using the concrete type support handle's type-erased pointer, ``const void * data``, we get type support-specific information.
For example, for C++ dynamic type support, this will be a ``const rosidl_typesupport_introspection_cpp::MessageMembers *``, which contains information about each field of the message.
See `this example function <https://github.com/christophebedard/rmw_email/blob/f5e622bab24edaad8e0da054c7dbc698c6fb809c/rmw_email_cpp/src/conversion.cpp#L116-L153>`__, which extracts language-dependent type support information from the concrete type support handle.
The information is used to read the type-erased message pointer and convert the message to a YAML object and then convert that to a string for the underlying middleware to publish.

Service type support is similar, but ``rosidl_service_type_support_t`` points to separate type support information for the request and response message types.

Tests
-----

The ``rmw`` package contains some tests, but they are mostly for utilities (e.g., getting zero-initialized structs) and non-implementation-specific functions such as topic/node name/namespace validation.

As for testing the new ``rmw`` implementation, the ``test_rmw_implementation`` package `contains tests for the interface <https://github.com/ros2/rmw_implementation/tree/{DISTRO}/test_rmw_implementation/test>`_.
Test executables are defined first and then a CMake function creates test targets for a given ``rmw`` implementation by setting the ``RMW_IMPLEMENTATION`` environment variable.
``rmw_implementation_cmake``'s ``call_for_each_rmw_implementation()`` is called and is provided with this CMake function, which is called with each available implementation.
See the `CMakeLists.txt file <https://github.com/ros2/rmw_implementation/blob/{DISTRO}/test_rmw_implementation/CMakeLists.txt>`__.
Many other packages, including the ``test_rclcpp`` test-only package, also `use this mechanism <https://github.com/ros2/system_tests/blob/{DISTRO}/test_rclcpp/CMakeLists.txt>`__ to run tests against all available ``rmw`` implementations, otherwise tests are simply run with the default implementation.
Packages can also use ``get_available_rmw_implementations()`` to get the actual list of available implementations.

Some tests have implementation-specific code, which is done for various reasons, such as unsupported interface subsets.
These tests can use ``rmw``'s ``rmw_get_implementation_identifier()`` `function <https://docs.ros.org/en/{DISTRO}/p/rmw/generated/function_rmw_8h_1aeb8a815b9be5eb3f38ab28363ef63920.html>`__ for this.

Middleware- and ``rmw`` implementation-specific configuration
-------------------------------------------------------------

The ``rmw`` interface allows providing arbitrary implementation-specific configuration payloads for publishers and subscriptions through the type-erased ``rmw_specific_publisher_payload`` / ``rmw_specific_subscription_payload`` fields in ``rmw_publisher_options_t`` / ``rmw_subscription_options_t``.
This is set by users through ``RMWImplementationSpecificPublisherPayload`` / ``RMWImplementationSpecificSubscriptionPayload`` in ``rclcpp``, for example.
This is an advanced, non-portable feature that is not currently used by any (tier 1) implementations.

For a bit more flexibility, some implementations use environment variables: ``RMW_FASTRTPS_*``, ``RMW_CONNEXT_*``, etc.
The underlying middleware may also be configurable through environment variables: ``FASTDDS_*``, ``ZENOH_*``, ``CYCLONEDDS_*``, ``EMAIL_*``, etc.
For example, the ``CYCLONEDDS_URI``, ``FASTDDS_DEFAULT_PROFILES_FILE``, and ``ZENOH_SESSION_CONFIG_URI`` environment variables can be used to provide a path to a full configuration file if the relevant middleware is used.

Footnotes
---------

.. [#fn_dds_rpc]
    There now is a DDS RPC specification, but `it was not finalized and wasn't implemented by DDS vendors back when ROS 2 was initially designed <https://design.ros2.org/articles/ros_on_dds.html#services-and-actions>`_.
    Since the ``rmw`` interface is also officially DDS-agnostic, services are up to the ``rmw`` implementation, which explains why :ref:`cross-DDS vendor communications are not guaranteed <different-middleware-vendors-cross-vendor-communication>`, even if pub/sub generally works.

.. [#fn_gid_remote_matching]
    In practice, this is not always the case, so this requirement is somewhat relaxed.
    See `ros2/rmw_cyclonedds#377 <https://github.com/ros2/rmw_cyclonedds/issues/377>`_.

.. [#fn_ts_fastrtps]
    For example, the C++ message Fast CDR serialization/deserialization code generated for {interface(std_msgs/msg/Header)} is at ``std_msgs/rosidl_typesupport_fastrtps_cpp/std_msgs/msg/detail/dds_fastrtps/header__type_support.cpp`` under the ``build/`` directory.

.. [#fn_ts_dynamic]
    For example, the C++ message introspection code generated for {interface(std_msgs/msg/Header)} is at ``std_msgs/rosidl_typesupport_introspection_cpp/std_msgs/msg/detail/header__type_support.cpp`` under the ``build/`` directory.
