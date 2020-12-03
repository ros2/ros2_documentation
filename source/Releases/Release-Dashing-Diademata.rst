
ROS 2 Dashing Diademata (codename 'dashing'; May 31st, 2019)
============================================================

.. contents:: Table of Contents
   :depth: 2
   :local:

*Dashing Diademata* is the fourth release of ROS 2.

Supported Platforms
-------------------

Dashing Diademata is primarily supported on the following platforms:

Tier 1 platforms:

* Ubuntu 18.04 (Bionic): ``amd64`` and ``arm64``
* Mac macOS 10.12 (Sierra)
* Windows 10 (Visual Studio 2019)

Tier 2 platforms:

* Ubuntu 18.04 (Bionic): ``arm32``

Tier 3 platforms:

* Debian Stretch (9): ``amd64``, ``arm64`` and ``arm32``
* OpenEmbedded Thud (2.6) / webOS OSE: ``arm32`` and ``x86``

For more information about RMW implementations, compiler / interpreter versions, and system dependency versions see `REP 2000 <https://www.ros.org/reps/rep-2000.html#dashing-diademata-may-2019-may-2021>`__.

Installation
------------

`Install Dashing Diademata <../dashing/Installation/Summary.html>`__

New features in this ROS 2 release
----------------------------------

A few features and improvements we would like to highlight:

* `Components <https://index.ros.org/doc/ros2/Tutorials/Composition/>`__ are now the recommended way to write your node.
  They can be used standalone as well as being composed within a process and both ways are fully support from ``launch`` files.
* The `intra-process communication <https://github.com/ros2/ros2_documentation/edit/master/source/Tutorials/Intra-Process-Communication.rst>`__ (C++ only) has been improved - both in terms of latency as well as minimizing copies.
* The Python client library has been updated to match most of the C++ equivalent and some important bug fixes and improvements have landed related to memory usage and performance.
* Parameters are now a complete alternative to ``dynamic_reconfigure`` from ROS 1 including constraints like ranges or being read-only.
* By relying on (a subset of) `IDL 4.2 <https://www.omg.org/spec/IDL/4.2>`__ for the message generation pipeline it is now possible to use ``.idl`` files (beside ``.msg`` / ``.srv`` / ``.action`` files).
  This change comes with support for optional UTF-8 encoding for ordinary strings as well as UTF-16 encoded multi-byte strings (see `wide strings design article <https://design.ros2.org/articles/wide_strings.html>`__).
* Command line tools related to ``actions`` and ``components``.
* Support for Deadline, Lifespan & Liveliness quality of service settings.
* MoveIt 2 `alpha release <https://github.com/AcutronicRobotics/moveit2/releases/tag/moveit_2_alpha>`__.

Please see the `Dashing meta ticket <https://github.com/ros2/ros2/issues/607>`__ on GitHub, which contains more information as well as references to specific tickets with additional details.


Changes since the Crystal release
---------------------------------

Declaring Parameters
^^^^^^^^^^^^^^^^^^^^

There have been some changes to the behavior of parameters starting in Dashing, which have also lead to some new API's and the deprecation of other API's.
See the ``rclcpp`` and ``rclpy`` sections below for more information about API changes.

Getting and Setting Undeclared Parameters
"""""""""""""""""""""""""""""""""""""""""

As of Dashing, parameters now need to be declared before being accessed or set.

Before Dashing, you could call ``get_parameter(name)`` and get either a value, if it had been previously set, or a parameter of type ``PARAMETER_NOT_SET``.
You could also call ``set_parameter(name, value)`` at any point, even if the parameter was previously unset.

Since Dashing, you need to first declare a parameter before getting or setting it.
If you try to get or set an undeclared parameter you will either get an exception thrown, e.g. ParameterNotDeclaredException, or in certain cases you will get an unsuccessful result communicated in a variety of ways (see specific functions for more details).

However, you can get the old behavior (mostly, see the note in the next paragraph) by using the ``allow_undeclared_parameters`` option when creating your node.
You might want to do this in order to avoid code changes for now, or in order to fulfill some uncommon use cases.
For example, a "global parameter server" or "parameter blackboard" may want to allow external nodes to set new parameters on itself without first declaring them, so it may use the ``allow_undeclared_parameters`` option to accomplish that.
In most cases, however, this option is not recommended because it makes the rest of the parameter API less safe to bugs like parameter name typos and "use before set" logical errors.

Note that using ``allow_undeclared_parameters`` will get you most of the old behavior specifically for "get" and "set" methods, but it will not revert all the behavior changes related to parameters back to how it was for ROS Crystal.
For that you need to also set the ``automatically_declare_parameters_from_overrides`` option to ``true``, which is described below in :ref:`Parameter Configuration using a YAML File <parameter-configuration-using-a-yaml-file>`.

Declaring a Parameter with a ParameterDescriptor
""""""""""""""""""""""""""""""""""""""""""""""""

Another benefit to declaring your parameters before using them, is that it allows you to declare a parameter descriptor at the same time.

Now when declaring a parameter you may include a custom ``ParameterDescriptor`` as well as a name and default value.
The ``ParameterDescriptor`` is defined as a message in ``rcl_interfaces/msg/ParameterDescriptor`` and contains meta data like ``description`` and constraints like ``read_only`` or ``integer_range``.
These constraints can be used to reject invalid values when setting parameters and/or as hints to external tools about what values are valid for a given parameter.
The ``read_only`` constraint will prevent the parameter's value from changing after being declared, as well as prevent if from being undeclared.

For reference, here's a link to the ``ParameterDescriptor`` message as of the time of writing this:

https://github.com/ros2/rcl_interfaces/blob/0aba5a142878c2077d7a03977087e7d74d40ee68/rcl_interfaces/msg/ParameterDescriptor.msg#L1

.. _parameter-configuration-using-a-yaml-file:

Parameter Configuration using a YAML File
"""""""""""""""""""""""""""""""""""""""""

As of Dashing, parameters in a YAML configuration file, e.g. passed to the node via the command line argument ``__params:=``, are only used to override a parameter's default value when declaring the parameter.

Before Dashing, any parameters you passed via a YAML file would be implicitly set on the node.

Since Dashing, this is no longer the case, as parameters need to be declared in order to appear on the node to external observers, like ``ros2 param list``.

The old behavior may be achieved using the ``automatically_declare_parameters_from_overrides`` option when creating a node.
This option, if set to ``true``, will automatically declare all parameters in the input YAML file when the node is constructed.
This may be used to avoid major changes to your existing code or to serve specific use cases.
For example, a "global parameter server" may want to be seeded with arbitrary parameters on launch, which it could not have declared ahead of time.
Most of the time, however, this option is not recommended, as it may lead to setting a parameter in a YAML file with the assumption that the node will use it, even if the node does not actually use it.

In the future we hope to have a checker that will warn you if you pass a parameter to a node that it was not expecting.

The parameters in the YAML file will continue to influence the value of parameters when they are first declared.

ament_cmake
^^^^^^^^^^^

The CMake function ``ament_index_has_resource`` was returning either ``TRUE`` or ``FALSE``.
As of `this release <https://github.com/ament/ament_cmake/pull/155>`_ it returns either the prefix path in case the resource was found or ``FALSE``.

If you are using the return value in a CMake condition like this:

.. code-block:: cmake

   ament_index_has_resource(var ...)
   if(${var})

you need to update the condition to ensure it considers a string value as ``TRUE``:

.. code-block:: cmake

   if(var)

rclcpp
^^^^^^

Behavior Change for ``Node::get_node_names()``
""""""""""""""""""""""""""""""""""""""""""""""

The function ``NodeGraph::get_node_names()``, and therefore also ``Node::get_node_names()``, now returns a ``std::vector<std::string>`` containing fully qualified node names with their namespaces included, instead of just the node names.

Changed the Way that Options are Passed to Nodes
""""""""""""""""""""""""""""""""""""""""""""""""

Extended arguments (beyond name and namespace) to the ``rclcpp::Node()`` constructor have been replaced with a ``rclcpp::NodeOptions`` structure.
See `ros2/rclcpp#622 <https://github.com/ros2/rclcpp/pull/622/files>`__ for details about the structure and default values of the options.

If you are using any of the extended arguments to ``rclcpp::Node()`` like this:

.. code-block:: cpp

  auto context = rclcpp::contexts::default_context::get_global_default_context();
  std::vector<std::string> args;
  std::vector<rclcpp::Parameter> params = { rclcpp::Parameter("use_sim_time", true) };
  auto node = std::make_shared<rclcpp::Node>("foo_node", "bar_namespace", context, args, params);

You need to update to use the ``NodeOptions`` structure

.. code-block:: cpp

  std::vector<std::string> args;
  std::vector<rclcpp::Parameter> params = { rclcpp::Parameter("use_sim_time", true) };
  rclcpp::NodeOptions node_options;
  node_options.arguments(args);
  node_options.parameter_overrides(params);
  auto node = std::make_shared<rclcpp::Node>("foo_node", "bar_namespace", node_options);

Changes to Creating Publishers and Subscriptions
""""""""""""""""""""""""""""""""""""""""""""""""

There have been a few changes to creating publishers and subscriptions which are new in Dashing:

- QoS settings are now passed using the new ``rclcpp::QoS`` class, and the API encourages the user to specify at least the history depth.
- Options are now passed as an object, i.e. ``rclcpp::PublisherOptions`` and ``rclcpp::SubscriptionOptions``.

All changes are backwards compatible (no code changes are required), but several existing call styles have been deprecated.
Users are encouraged to update to the new signatures.

----

In the past, when creating a publisher or subscription, you could either not specify any QoS settings (e.g. just provide topic name for a publisher) or you could specify a "qos profile" data structure (of type ``rmw_qos_profile_t``) with all the settings already set.
Now you must use the new ``rclcpp::QoS`` object to specify your QoS and at least the history settings for your QoS.
This encourages the user to specify a history depth when using ``KEEP_LAST``, rather than defaulting it to a value that may or may not be appropriate.

In ROS 1, this was known as the ``queue_size`` and it was required in both C++ and Python.
We're changing the ROS 2 API to bring this requirement back.

----

Also, any options which could previously be passed during creation of a publisher or subscription have now been encapsulated in an ``rclcpp::PublisherOptions`` and ``rclcpp::SubscriptionOptions`` class respectively.
This allows for shorter signatures, more convenient use, and for adding new future options without breaking API.

----

Some signatures for creating publishers and subscribers are now deprecated, and new signatures have been added to allow you to use the new ``rclcpp::QoS`` and publisher/subscription option classes.

These are the new and recommended API's:

.. code-block:: cpp

  template<
    typename MessageT,
    typename AllocatorT = std::allocator<void>,
    typename PublisherT = ::rclcpp::Publisher<MessageT, AllocatorT>>
  std::shared_ptr<PublisherT>
  create_publisher(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    const PublisherOptionsWithAllocator<AllocatorT> & options =
    PublisherOptionsWithAllocator<AllocatorT>()
  );

  template<
    typename MessageT,
    typename CallbackT,
    typename AllocatorT = std::allocator<void>,
    typename SubscriptionT = rclcpp::Subscription<
      typename rclcpp::subscription_traits::has_message_type<CallbackT>::type, AllocatorT>>
  std::shared_ptr<SubscriptionT>
  create_subscription(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    CallbackT && callback,
    const SubscriptionOptionsWithAllocator<AllocatorT> & options =
    SubscriptionOptionsWithAllocator<AllocatorT>(),
    typename rclcpp::message_memory_strategy::MessageMemoryStrategy<
      typename rclcpp::subscription_traits::has_message_type<CallbackT>::type, AllocatorT
    >::SharedPtr
    msg_mem_strat = nullptr);

And these are the deprecated ones:

.. code-block:: cpp

  template<
    typename MessageT,
    typename AllocatorT = std::allocator<void>,
    typename PublisherT = ::rclcpp::Publisher<MessageT, AllocatorT>>
  [[deprecated("use create_publisher(const std::string &, const rclcpp::QoS &, ...) instead")]]
  std::shared_ptr<PublisherT>
  create_publisher(
    const std::string & topic_name,
    size_t qos_history_depth,
    std::shared_ptr<AllocatorT> allocator);

  template<
    typename MessageT,
    typename AllocatorT = std::allocator<void>,
    typename PublisherT = ::rclcpp::Publisher<MessageT, AllocatorT>>
  [[deprecated("use create_publisher(const std::string &, const rclcpp::QoS &, ...) instead")]]
  std::shared_ptr<PublisherT>
  create_publisher(
    const std::string & topic_name,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_default,
    std::shared_ptr<AllocatorT> allocator = nullptr);

  template<
    typename MessageT,
    typename CallbackT,
    typename Alloc = std::allocator<void>,
    typename SubscriptionT = rclcpp::Subscription<
      typename rclcpp::subscription_traits::has_message_type<CallbackT>::type, Alloc>>
  [[deprecated(
    "use create_subscription(const std::string &, const rclcpp::QoS &, CallbackT, ...) instead"
  )]]
  std::shared_ptr<SubscriptionT>
  create_subscription(
    const std::string & topic_name,
    CallbackT && callback,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_default,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr,
    bool ignore_local_publications = false,
    typename rclcpp::message_memory_strategy::MessageMemoryStrategy<
      typename rclcpp::subscription_traits::has_message_type<CallbackT>::type, Alloc>::SharedPtr
    msg_mem_strat = nullptr,
    std::shared_ptr<Alloc> allocator = nullptr);

  template<
    typename MessageT,
    typename CallbackT,
    typename Alloc = std::allocator<void>,
    typename SubscriptionT = rclcpp::Subscription<
      typename rclcpp::subscription_traits::has_message_type<CallbackT>::type, Alloc>>
  [[deprecated(
    "use create_subscription(const std::string &, const rclcpp::QoS &, CallbackT, ...) instead"
  )]]
  std::shared_ptr<SubscriptionT>
  create_subscription(
    const std::string & topic_name,
    CallbackT && callback,
    size_t qos_history_depth,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr,
    bool ignore_local_publications = false,
    typename rclcpp::message_memory_strategy::MessageMemoryStrategy<
      typename rclcpp::subscription_traits::has_message_type<CallbackT>::type, Alloc>::SharedPtr
    msg_mem_strat = nullptr,
    std::shared_ptr<Alloc> allocator = nullptr);

----

The change to how QoS is passed is most likely to impact users.

A typical change for a publisher looks like this:

.. code-block:: diff

  - pub_ = create_publisher<std_msgs::msg::String>("chatter");
  + pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);

And for a subscription:

.. code-block:: diff

  - sub_ = create_subscription<std_msgs::msg::String>("chatter", callback);
  + sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, callback);

If you have no idea what depth to use and don't care right now (maybe just prototyping), then we recommend using ``10``, as that was the default before and should preserve existing behavior.

More in depth documentation about how to select an appropriate depth is forthcoming.

This is an example of a slightly more involved change to avoid the newly deprecated API's:

.. code-block:: diff

  - // Creates a latched topic
  - rmw_qos_profile_t qos = rmw_qos_profile_default;
  - qos.depth = 1;
  - qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  -
    model_xml_.data = model_xml;
    node_handle->declare_parameter("robot_description", model_xml);
    description_pub_ = node_handle->create_publisher<std_msgs::msg::String>(
  -   "robot_description", qos);
  +   "robot_description",
  +   // Transient local is similar to latching in ROS 1.
  +   rclcpp::QoS(1).transient_local());

See the pull request (and connected pull requests) that introduced the QoS change for more examples and details:

- https://github.com/ros2/rclcpp/pull/713

  - https://github.com/ros2/demos/pull/332
  - https://github.com/ros2/robot_state_publisher/pull/19
  - and others...


Changes Due to Declare Parameter Change
"""""""""""""""""""""""""""""""""""""""

For details about the actual behavior change, see `Declaring Parameters`_ above.

There are several new API calls in the ``rclcpp::Node``'s interface:

- Methods that declare parameters given a name, optional default value, optional descriptor, and return the value actually set:

  .. code-block:: c++

    const rclcpp::ParameterValue &
    rclcpp::Node::declare_parameter(
      const std::string & name,
      const rclcpp::ParameterValue & default_value = rclcpp::ParameterValue(),
      const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
      rcl_interfaces::msg::ParameterDescriptor());

    template<typename ParameterT>
    auto
    rclcpp::Node::declare_parameter(
      const std::string & name,
      const ParameterT & default_value,
      const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
      rcl_interfaces::msg::ParameterDescriptor());

    template<typename ParameterT>
    std::vector<ParameterT>
    rclcpp::Node::declare_parameters(
      const std::string & namespace_,
      const std::map<std::string, ParameterT> & parameters);

    template<typename ParameterT>
    std::vector<ParameterT>
    rclcpp::Node::declare_parameters(
      const std::string & namespace_,
      const std::map<
        std::string,
        std::pair<ParameterT, rcl_interfaces::msg::ParameterDescriptor>
      > & parameters);

- A method to undeclare parameters and to check if a parameter has been declared:

  .. code-block:: c++

    void
    rclcpp::Node::undeclare_parameter(const std::string & name);

    bool
    rclcpp::Node::has_parameter(const std::string & name) const;

- Some convenience methods that did not previously exist:

  .. code-block:: c++

    rcl_interfaces::msg::SetParametersResult
    rclcpp::Node::set_parameter(const rclcpp::Parameter & parameter);

    std::vector<rclcpp::Parameter>
    rclcpp::Node::get_parameters(const std::vector<std::string> & names) const;

    rcl_interfaces::msg::ParameterDescriptor
    rclcpp::Node::describe_parameter(const std::string & name) const;

- A new method to set the callback which is called anytime a parameter will be changed, giving you the opportunity to reject it:

  .. code-block:: c++

    using OnParametersSetCallbackType =
      rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType;

    OnParametersSetCallbackType
    rclcpp::Node::set_on_parameters_set_callback(
      OnParametersSetCallbackType callback);

There were also several deprecated methods:

  .. code-block:: c++

    template<typename ParameterT>
    [[deprecated("use declare_parameter() instead")]]
    void
    rclcpp::Node::set_parameter_if_not_set(
      const std::string & name,
      const ParameterT & value);

    template<typename ParameterT>
    [[deprecated("use declare_parameters() instead")]]
    void
    rclcpp::Node::set_parameters_if_not_set(
      const std::string & name,
      const std::map<std::string, ParameterT> & values);

    template<typename ParameterT>
    [[deprecated("use declare_parameter() and it's return value instead")]]
    void
    rclcpp::Node::get_parameter_or_set(
      const std::string & name,
      ParameterT & value,
      const ParameterT & alternative_value);

    template<typename CallbackT>
    [[deprecated("use set_on_parameters_set_callback() instead")]]
    void
    rclcpp::Node::register_param_change_callback(CallbackT && callback);

Memory Strategy
"""""""""""""""

The interface ``rclcpp::memory_strategy::MemoryStrategy`` was using the typedef ``WeakNodeVector`` in various method signatures.
As of Dashing the typedef has been been changed to ``WeakNodeList`` and with it the type of the parameter in various methods.
Any custom memory strategy needs to be updated to match the modified interface.

The relevant API change can be found in `ros2/rclcpp#741 <https://github.com/ros2/rclcpp/pull/741>`__.

rclcpp_components
^^^^^^^^^^^^^^^^^

The correct way to implement composition in Dashing is by utilizing the ``rclcpp_components`` package.

The following changes must be made to nodes in order to correctly implement runtime composition:

The Node must have a constructor that takes ``rclcpp::NodeOptions``:

.. code-block:: cpp

  class Listener: public rclcpp::Node {
    Listener(const rclcpp::NodeOptions & options)
    : Node("listener", options)
    {
    }
  };

C++ registration macros (if present) need to be updated to use the ``rclcpp_components`` equivalent.
If not present, registration macros must be added in one translation unit.

.. code-block:: cpp

  // Insert at bottom of translation unit, e.g. listener.cpp
  #include "rclcpp_components/register_node_macro.hpp"
  // Use fully-qualifed name in registration
  RCLCPP_COMPONENTS_REGISTER_NODE(composition::Listener);

CMake registration macros (if present) need to be updated.
If not present, registration macros must be added to the project's CMake.

.. code-block:: cmake

  add_library(listener src/listener.cpp)
  rclcpp_components_register_nodes(listener "composition::Listener")

For more information on composition, see `the tutorial <https://index.ros.org/doc/ros2/Tutorials/Composition/>`__

rclpy
^^^^^

Changes to Creating Publishers, Subscriptions, and QoS Profiles
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Prior to Dashing, you could optionally provide a ``QoSProfile`` object when creating a publisher or subscription.
In an effort to encourage users to specify a history depth for message queues, we now **require** that a depth value or ``QoSProfile`` object is given when creating publishers or subscriptions.

To create a publisher, previously you would have written:

.. code-block:: python

  node.create_publisher(Empty, 'chatter')
  # Or using a keyword argument for QoSProfile
  node.create_publisher(Empty, 'chatter', qos_profile=qos_profile_sensor_data)

In Dashing, prefer the following API that provides a depth value or ``QoSProfile`` object as a third positional argument:

.. code-block:: python

  # Assume a history setting of KEEP_LAST with depth 10
  node.create_publisher(Empty, 'chatter', 10)
  # Or pass a QoSProfile object directly
  node.create_publisher(Empty, 'chatter', qos_profile_sensor_data)

Likewise for subscriptions, previously you would have written:

.. code-block:: python

  node.create_subscription(BasicTypes, 'chatter', lambda msg: print(msg))
  # Or using a keyword argument for QoSProfile
  node.create_subscription(BasicTypes, 'chatter', lambda msg: print(msg), qos_profile=qos_profile_sensor_data)

In Dashing:

.. code-block:: python

  # Assume a history setting of KEEP_LAST with depth 10
  node.create_subscription(BasicTypes, 'chatter', lambda msg: print(msg), 10)
  # Or pass a QoSProfile object directly
  node.create_subscription(BasicTypes, 'chatter', lambda msg: print(msg), qos_profile_sensor_data)

To ease the transition, users who do not use the new API will see deprecation warnings.

Furthermore, we also require that when constructing ``QoSProfile`` objects that a history policy and/or depth is set.
If a history policy of ``KEEP_LAST`` is provided, then a depth argument is also required.
For example, these calls are valid:

.. code-block:: python

  QoSProfile(history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL)
  QoSProfile(history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth=10)
  QoSProfile(depth=10)  # equivalent to the previous line

And these calls will cause a deprecation warning:

.. code-block:: python

  QoSProfile()
  QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
  # KEEP_LAST but no depth
  QoSProfile(history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST)

See the issue and pull request related to introducing this change for more details:

- https://github.com/ros2/rclpy/issues/342
- https://github.com/ros2/rclpy/pull/344


Changes Due to Declare Parameter Change
"""""""""""""""""""""""""""""""""""""""

For details about the actual behavior change, see `Declaring Parameters`_ above. The changes are analogous to the ones in ``rclcpp``.

These are the new API methods available in ``rclpy.node.Node`` interface:

- To declare parameters given a name, an optional default value (supported by ``rcl_interfaces.msg.ParameterValue``) and an optional descriptor, returning the value actually set:

  .. code-block:: python

      def declare_parameter(
          name: str,
          value: Any = None,
          descriptor: ParameterDescriptor = ParameterDescriptor()
      ) -> Parameter

      def declare_parameters(
        namespace: str,
        parameters: List[Union[
            Tuple[str],
            Tuple[str, Any],
            Tuple[str, Any, ParameterDescriptor],
        ]]
      ) -> List[Parameter]

- To undeclare previously declared parameters and to check if a parameter has been declared beforehand:

  .. code-block:: python

      def undeclare_parameter(name: str) -> None

      def has_parameter(name: str) -> bool

- To get and set parameter descriptors:

  .. code-block:: python

      def describe_parameter(name: str) -> ParameterDescriptor

      def describe_parameters(names: List[str]) -> List[ParameterDescriptor]

      def set_descriptor(
          name: str,
          descriptor: ParameterDescriptor,
          alternative_value: Optional[ParameterValue] = None
      ) -> ParameterValue

- A convenience method to get parameters that may not have been declared:

  .. code-block:: python

      def get_parameter_or(name: str, alternative_value: Optional[Parameter] = None) -> Parameter

Other changes
"""""""""""""

``rclpy.parameter.Parameter`` can now guess its type without explicitly setting it (as long as it's one of the supported ones by ``rcl_interfaces.msg.ParameterValue``).
For example, this code:

  .. code-block:: python

      p = Parameter('myparam', Parameter.Type.DOUBLE, 2.41)

Is equivalent to this code:

  .. code-block:: python

      p = Parameter('myparam', value=2.41)

This change does not break existing API.

rosidl
^^^^^^

Until Crystal each message generator package registered itself using the ``ament_cmake`` extension point ``rosidl_generate_interfaces`` and was passed a set of ``.msg`` / ``.srv`` / ``.action`` files.
As of Dashing the message generation pipeline is based on ``.idl`` files instead.

Any message generator package needs to change and register itself using the new extension point ``rosidl_generate_idl_interfaces`` which passes only ``.idl`` files instead.
The message generators for the commonly supported languages C, C++, and Python as well as the typesupport packages for introspection, Fast RTPS, Connext and OpenSplice have already been updated (see `ros2/rosidl#334 <https://github.com/ros2/rosidl/pull/334/files>`__).
The CMake code calling ``rosidl_generate_interfaces()`` can either pass ``.idl`` files directly or pass ``.msg`` / ``.srv`` / ``.action`` which will then internally be converted into ``.idl`` files before being passed to each message generator.

The format of ``.msg`` / ``.srv`` / ``.action`` files is not being evolved in the future.
The mapping between ``.msg`` / ``.srv`` / ``.action`` files and ``.idl`` files is described in `this design article <https://design.ros2.org/articles/legacy_interface_definition.html>`__.
A `second design article <https://design.ros2.org/articles/idl_interface_definition.html>`__ describes the supported features in ``.idl`` files.
In order to leverage any of the new features existing interfaces need to be converted (e.g. using the command line tools  ``msg2idl`` / ``srv2idl`` / ``action2idl``).

To distinguish same type names, but with different namespaces, the introspection structs now contain a namespace field that replaces the package name (see `ros2/rosidl#335 <https://github.com/ros2/rosidl/pull/355/files>`_).

Mapping of char in .msg files
"""""""""""""""""""""""""""""

In `ROS 1 <https://wiki.ros.org/msg#Fields>`__ ``char`` has been deprecated for a long time and is being mapped to ``uint8``.
In ROS 2 until Crystal ``char`` was mapped to a single character (``char`` in C / C++, ``str`` with length 1 in Python) in an effort to provide a more natural mapping.
As of Dashing the ROS 1 semantic has been restored and ``char`` maps to ``uint8`` again.

rosidl_generator_cpp
^^^^^^^^^^^^^^^^^^^^

The C++ data structures generated for messages, services and actions provide setter methods for each field.
Until Crystal each setter returned a pointer to the data structure itself to enable the named parameter idiom.
As of Dashing these setters `return a reference <https://github.com/ros2/rosidl/pull/353>`__ instead since that seems to be the more common signature as well as it clarifies that the returned value can't be a ``nullptr``.

rosidl_generator_py
^^^^^^^^^^^^^^^^^^^

Until Crystal an array (fixed size) or sequence (dynamic size, optionally with an upper boundary) field in a message was stored as a ``list`` in Python.
As of Dashing the Python type for arrays / sequences of numeric values has been changed:

* an array of numeric values is stored as a ``numpy.ndarray`` (the ``dtype`` is chosen to match the type of the numeric value)
* a sequence of numeric values is stored as an ``array.array`` (the ``typename`` is chosen to match the type of the numeric value)

As before an array / sequence of non-numeric types is still represented as a ``list`` in Python.

This change brings a number of benefits:

* The new data structures ensure that each item in the array / sequence complies with the value range restrictions of the numeric type.
* The numeric values can be stored more efficiently in memory which avoid the overhead of Python objects for each item.
* The memory layout of both data structures allows to read and write all items of the array / sequence in a single operation which makes the conversion from and to Python significantly faster / more efficient.

launch
^^^^^^

The ``launch_testing`` package caught up with the ``launch`` package redesign done in Bouncy Bolson.
The legacy Python API, already moved into the ``launch.legacy`` submodule, has thus been deprecated and removed.

See ``launch`` `examples <https://github.com/ros2/launch/tree/master/launch/examples>`__ and `documentation <https://github.com/ros2/launch/tree/master/launch/doc>`__ for reference on how to use its new API.

See `demos tests <https://github.com/ros2/demos>`__ for reference on how to use the new ``launch_testing`` API.

rmw
^^^

Changes since the `Crystal Clemmys <Release-Crystal-Clemmys>` release:

* New API in ``rmw``, a fini function for ``rmw_context_t``:

 * `rmw_context_fini <https://github.com/ros2/rmw/blob/c518842f6f82910482470b40c221c268d30691bd/rmw/include/rmw/init.h#L111-L136>`_

* Modification of ``rmw``, now passes ``rmw_context_t`` to ``rmw_create_wait_set``:

 * `rmw_create_wait_set <https://github.com/ros2/rmw/blob/c518842f6f82910482470b40c221c268d30691bd/rmw/include/rmw/rmw.h#L522-L543>`_

* New APIs in ``rmw`` for preallocating space for published and subscribed messages:

 * `rmw_init_publisher_allocation <https://github.com/ros2/rmw/blob/dc7b2f49f1f961d6cf2c173adc54736451be8938/rmw/include/rmw/rmw.h#L262>`_
 * `rmw_fini_publisher_allocation <https://github.com/ros2/rmw/blob/dc7b2f49f1f961d6cf2c173adc54736451be8938/rmw/include/rmw/rmw.h#L279>`_
 * `rmw_init_subscription_allocation <https://github.com/ros2/rmw/blob/dc7b2f49f1f961d6cf2c173adc54736451be8938/rmw/include/rmw/rmw.h#L489>`_
 * `rmw_fini_subscription_allocation <https://github.com/ros2/rmw/blob/dc7b2f49f1f961d6cf2c173adc54736451be8938/rmw/include/rmw/rmw.h#L506>`_
 * `rmw_serialized_message_size <https://github.com/ros2/rmw/blob/dc7b2f49f1f961d6cf2c173adc54736451be8938/rmw/include/rmw/rmw.h#L395>`_

* Modification of ``rmw``, now passes ``rmw_publisher_allocation_t`` or ``rmw_subscription_allocation_t`` to ``rmw_publish`` and ``rmw_take``, respectively.
  Note that this argument can be ``NULL`` or ``nullptr``, which keeps existing Crystal behavior.

 * `rmw_publish <https://github.com/ros2/rmw/blob/dc7b2f49f1f961d6cf2c173adc54736451be8938/rmw/include/rmw/rmw.h#L310>`_
 * `rmw_take <https://github.com/ros2/rmw/blob/dc7b2f49f1f961d6cf2c173adc54736451be8938/rmw/include/rmw/rmw.h#L556>`_

* Type names returned by ``rmw_get_*_names_and_types*`` functions should have a fully-qualified namespace.
  For example, instead of ``rcl_interfaces/Parameter`` and ``rcl_interfaces/GetParameters``, the returned type names should be ``rcl_interface/msg/Parameter`` and ``rcl_interfaces/srv/GetParameters``.

actions
^^^^^^^

* Changes to ``rclcpp_action::Client`` signatures:

  The signature of `rclcpp_action::Client::async_send_goal <https://github.com/ros2/rclcpp/blob/ef41059a751702274667e2164182c062b47c453d/rclcpp_action/include/rclcpp_action/client.hpp#L343>`_ has changed.
  Now users can optionally provide callback functions for the **goal response** and the **result** using the new
  `SendGoalOptions <https://github.com/ros2/rclcpp/blob/ef41059a751702274667e2164182c062b47c453d/rclcpp_action/include/rclcpp_action/client.hpp#L276>`_ struct.
  The goal response callback is called when an action server accepts or rejects the goal and the result callback is called when the result for the goal is received.
  Optional callbacks were also added to `rclcpp_action::Client::async_cancel_goal <https://github.com/ros2/rclcpp/blob/ef41059a751702274667e2164182c062b47c453d/rclcpp_action/include/rclcpp_action/client.hpp#L432-L434>`_
  and `rclcpp_action::Client::async_get_result <https://github.com/ros2/rclcpp/blob/ef41059a751702274667e2164182c062b47c453d/rclcpp_action/include/rclcpp_action/client.hpp#L399-L401>`_.

* Changes to goal transition names:

  The names of goal state transitions have been refactored to reflect the design documention.
  This affects ``rcl_action``, ``rclcpp_action``, and ``rclpy``.
  Here is a list of the event name changes (*Old name -> New name*):

  * GOAL_EVENT_CANCEL -> GOAL_EVENT_CANCEL_GOAL
  * GOAL_EVENT_SET_SUCCEEDED -> GOAL_EVENT_SUCCEED
  * GOAL_EVENT_SET_ABORTED -> GOAL_EVENT_ABORT
  * GOAL_EVENT_SET_CANCELED -> GOAL_EVENT_CANCELED

* Changes to ``CancelGoal.srv``:

  A ``return_code`` field was added to the response message of the ``CancelGoal`` service.
  This is to better communicate a reason for a failed service call.
  See the `pull request <https://github.com/ros2/rcl_interfaces/pull/76>`_ and connected issue for details.

rviz
^^^^

* Plugins should use fully qualified type names otherwise a warning will be logged.
  For `example <https://github.com/ros2/rviz/blob/dfceae319d49546f1e4ad39689853c18fef0001e/rviz_default_plugins/plugins_description.xml#L13>`_, use the type ``sensor_msgs/msg/Image`` instead of ``sensor_msgs/Image``.
  See `PR introducing this change <https://github.com/ros2/rviz/pull/387>`_ for more details.

Known Issues
------------

* `[ros2/rclcpp#715] <https://github.com/ros2/rclcpp/issues/715>`_ There is an inconsistency in the way that parameter YAML files are loaded between standalone ROS 2 nodes and composed ROS 2 nodes.
  Currently available workarounds are noted in an `issue comment <https://github.com/ros2/rclcpp/issues/715#issuecomment-497392626>`_
* `[ros2/rclpy#360] <https://github.com/ros2/rclpy/issues/360>`_ rclpy nodes ignore ``ctrl-c`` when using OpenSplice on Windows.
* `[ros2/rosidl_typesupport_opensplice#30] <https://github.com/ros2/rosidl_typesupport_opensplice/issues/30>`_ There is a bug preventing nesting a message inside of a service or action definition with the same name when using OpenSplice.
* `[ros2/rclcpp#781] <https://github.com/ros2/rclcpp/pull/781>`_ Calling ``get_parameter``/``list_parameter`` from within ``on_set_parameter_callback`` causes a deadlock on Dashing.  This is fixed for Eloquent, but is an ABI break so has not been backported to Dashing.
* `[ros2/rclcpp#912] <https://github.com/ros2/rclcpp/issues/912>`_ Inter-process communication forces a message copy when intra-process communication takes place between an ``std::unique_ptr`` publisher and a single ``std::unique_ptr`` subscription (published ``std::unique_ptr`` is internally being promoted to an ``std::shared_ptr``).
* `[ros2/rosbag2#125] <https://github.com/ros2/rosbag2/issues/125>`_ Topics with unreliable QOS are not recorded.
* `[ros2/rclcpp#715] <https://github.com/ros2/rclcpp/issues/715>`_ Composable nodes cannot receive parameters via remapping. Supplying parameters to composable nodes can be accomplished using the methods described in `[this comment] <https://github.com/ros2/rclcpp/issues/715#issuecomment-497392626>`_.
* `[ros2/rclcpp#893] <https://github.com/ros2/rclcpp/issues/893>`_ ``rclcpp::Context`` is not destroyed because of a reference cycle with ``rclcpp::GraphListener``. This causes a memory leak. A fix has not been backported because of the risk of breaking ABI.

Timeline before the release
---------------------------

A few milestones leading up to the release:

    Mon. Apr 8th (alpha)
        First releases of core packages available.
        Testing can happen from now on (some features might not have landed yet).

    Thu. May 2nd
        API freeze for core packages

    Mon. May 6th (beta)
        Updated releases of core packages available.
        Additional testing of the latest features.

    Thu. May 16th
        Feature freeze.
        Only bug fix releases should be made after this point.
        New packages can be released independently.

    Mon. May 20th (release candidate)
        Updated releases of core packages available.

    Wed. May 29th
        Freeze rosdistro.
        No PRs for Dashing on the `rosdistro` repo will be merged (reopens after the release announcement).
