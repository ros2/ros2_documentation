.. _upcoming-release:

.. _humble-release:

.. move this directive when next release page is created

ROS 2 Humble Hawksbill (codename 'humble'; May, 2022)
=====================================================

.. contents:: Table of Contents
   :depth: 2
   :local:

*Humble Hawksbill* is the eighth release of ROS 2.
What follows is highlights of the important changes and features in Humble Hawksbill since the last release.

Supported Platforms
-------------------

Humble Hawksbill is primarily supported on the following platforms:

Tier 1 platforms:

TBD

Tier 2 platforms:

TBD

Tier 3 platforms:

TBD

For more information about RMW implementations, compiler / interpreter versions, and system dependency versions see `REP 2000 <https://www.ros.org/reps/rep-2000.html>`__.

Installation
------------

To come.

New features in this ROS 2 release
----------------------------------

``ros_args`` attribute & ``ros_arguments`` parameter for nodes in launch files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is now possible to provide `ROS-specific node arguments <../How-To-Guides/Node-arguments>` directly, without needing to use ``args`` with a leading ``--ros-args`` flag:

.. tabs::

   .. group-tab:: XML

    .. code-block:: xml

      <launch>
        <node pkg="demo_nodes_cpp" exec="talker" ros_args="--log-level debug" />
      </launch>

   .. group-tab:: YAML

      .. code-block:: yaml

        launch:
        - node:
            pkg: demo_nodes_cpp
            exec: talker
            ros_args: '--log-level debug'

The corresponding parameter for the ``Node`` action in Python launch files is ``ros_arguments``:

.. code-block:: python

  from launch import LaunchDescription
  import launch_ros.actions

  def generate_launch_description():
      return LaunchDescription([
          launch_ros.actions.Node(
              package='demo_nodes_cpp',
              executable='talker',
              ros_arguments=['--log-level', 'debug'],
          ),
      ])

Related PRs: `ros2/launch_ros#249 <https://github.com/ros2/launch_ros/pull/249>`_ and `ros2/launch_ros#253 <https://github.com/ros2/launch_ros/pull/253>`_.

SROS2 Security enclaves now support Certificate Revocation Lists
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Certificate Revocation Lists (CRLs) are a concept where particular certificates can be revoked before their expiration.
As of Humble, it is now possible to put a CRL in an SROS2 security enclave and have it be honored.
See `the SROS2 tutorials <https://github.com/ros2/sros2/blob/master/SROS2_Linux.md#certificate-revocation-lists>`__ for an example of how to use it.

Changes since the Galactic release
----------------------------------

common_interfaces
^^^^^^^^^^^^^^^^^

Support Textures and Embedded Meshes for Marker Messages
""""""""""""""""""""""""""""""""""""""""""""""""""""""""

These two additions will improve the ability to both visualize data in new ways with standard messages and, simultaneously, enable the ability to track this data in rosbag.

**Textures** bring the addition of three new fields to markers:

.. code-block:: bash

   # Texture resource is a special URI that can either reference a texture file in
   # a format acceptable to (resource retriever)[https://index.ros.org/p/resource_retriever/]
   # or an embedded texture via a string matching the format:
   #   "embedded://texture_name"
   string texture_resource
   # An image to be loaded into the rendering engine as the texture for this marker.
   # This will be used iff texture_resource is set to embedded.
   sensor_msgs/CompressedImage texture
   # Location of each vertex within the texture; in the range: [0.0-1.0]
   UVCoordinate[] uv_coordinates

RViz will fully support texture rendering through the embedded format.

To those familiar with ``mesh_resource``, ``resource_retriever`` should be familiar. This will allow the programmer to choose where they want to load data from, either a local file or a networked file. In the interest of being able to record all data in a rosbag, the ability to embed the texture image is included.

**Meshes** were modified in a similar way to add the ability to embed a raw Mesh file for the purpose of recording and are modified in a similar way. The Meshfile message has two fields:

.. code-block:: bash

   # The filename is used for both debug purposes and to provide a file extension
   # for whatever parser is used.
   string filename

   # This stores the raw text of the mesh file.
   uint8[] data

The embedded ``Meshfile`` message is not yet supported in implementation.

Related PRs: `ros2/common_interfaces#153 <https://github.com/ros2/common_interfaces/pull/153>`_ `ros2/rviz#719 <https://github.com/ros2/rviz/pull/719>`_

rmw
^^^

``struct`` type name suffix changed from ``_t`` to ``_s``
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""

To avoid type name duplication errors between ``struct`` type names and their ``typedef``-ed aliases when generating code documentation, the suffix for all ``struct`` type names has been changed from ``_t`` to ``_s``. Aliases with ``_t`` suffixes remain in place. Thus, this change is a breaking change only for code that uses full ``struct`` type specifiers i.e. ``struct type_name_t``.

See `ros2/rmw#313 <https://github.com/ros2/rmw/pull/313>`__ for more details.

rcl
^^^

``struct`` type name suffix changed from ``_t`` to ``_s``
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""

To avoid type name duplication errors between ``struct`` type names and their ``typedef``-ed aliases when generating code documentation, the suffix for all ``struct`` type names has been changed from ``_t`` to ``_s``. Aliases with ``_t`` suffixes remain in place. Thus, this change is a breaking change only for code that uses full ``struct`` type specifiers i.e. ``struct type_name_t``.

See `ros2/rcl#932 <https://github.com/ros2/rcl/pull/932>`__ for more details.

rclcpp
^^^^^^

Support Type Adaption for Publishers and Subscriptions
""""""""""""""""""""""""""""""""""""""""""""""""""""""

After defining a type adapter, custom data structures can be used directly by publishers and subscribers, which helps to avoid additional work for the programmer and potential sources of errors.
This is especially useful when working with complex data types, such as when converting OpenCV's ``cv::Map`` to ROS's ``sensor_msgs/msg/Image`` type.

Here is an example of a type adapter that converts ``std_msgs::msg::String`` to ``std::string``:

.. code-block:: cpp

   template<>
   struct rclcpp::TypeAdapter<
      std::string,
      std_msgs::msg::String
   >
   {
     using is_specialized = std::true_type;
     using custom_type = std::string;
     using ros_message_type = std_msgs::msg::String;

     static
     void
     convert_to_ros_message(
       const custom_type & source,
       ros_message_type & destination)
     {
       destination.data = source;
     }

     static
     void
     convert_to_custom(
       const ros_message_type & source,
       custom_type & destination)
     {
       destination = source.data;
     }
   };

And an example of how the type adapter can be used:

.. code-block:: cpp

   using MyAdaptedType = TypeAdapter<std::string, std_msgs::msg::String>;

   // Publish a std::string
   auto pub = node->create_publisher<MyAdaptedType>(...);
   std::string custom_msg = "My std::string"
   pub->publish(custom_msg);

   // Pass a std::string to a subscription's callback
   auto sub = node->create_subscription<MyAdaptedType>(
     "topic",
     10,
     [](const std::string & msg) {...});

To learn more, see the `publisher <https://github.com/ros2/examples/blob/b83b18598b198b4a5ba44f9266c1bb39a393fa17/rclcpp/topics/minimal_publisher/member_function_with_type_adapter.cpp>`_ and `subscription <https://github.com/ros2/examples/blob/b83b18598b198b4a5ba44f9266c1bb39a393fa17/rclcpp/topics/minimal_subscriber/member_function_with_type_adapter.cpp>`_) examples, as well as a more complex `demo <https://github.com/ros2/demos/pull/482>`_.
For more details, see `REP 2007 <https://ros.org/reps/rep-2007.html>`_.

``get_callback_groups`` method removed from ``NodeBase`` and ``Node`` classes
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

``for_each_callback_group()`` method has replaced ``get_callback_groups()`` by providing a thread-safe way to access ``callback_groups_`` vector.
``for_each_callback_group()`` accepts a function as an argument, iterates over the stored callback groups, and calls the passed function to ones that are valid.

For more details, please refer to this `pull request <https://github.com/ros2/rclcpp/pull/1723>`_.

ros2cli
^^^^^^^

``ros2 topic pub`` will wait for one matching subscription when using ``--times/--once/-1``
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

When using ``--times/--once/-1`` flags, ``ros2 topic pub`` will wait for one matching subscription to be found before starting to publish.
This avoids the issue of the ros2cli node starting to publish before discovering a matching subscription, which results in some of the first messages being lost.
This is particularly unexpected when using a reliable qos profile.

The number of matching subscriptions to wait before starting publishing can be configured with the ``-w/--wait-matching-subscriptions`` flags, e.g.:

.. code-block:: console

   ros2 topic pub -1 -w 3 /chatter std_msgs/msg/String "{data: 'foo'}"


to wait for three matching subscriptions before starting to publish.

``-w`` can also be used independently of ``--times/--once/-1`` but it only defaults to one when combined with them, otherwise the ``-w`` default is zero.

See https://github.com/ros2/ros2cli/pull/642 for more details.

``ros2 param dump`` default output changed
""""""""""""""""""""""""""""""""""""""""""

  * ``--print`` option for dump command was `deprecated <https://github.com/ros2/ros2cli/pull/638>`_.

    It prints to stdout by default:

    .. code-block:: bash

      ros2 param dump /my_node_name

  * ``--output-dir`` option for dump command was `deprecated <https://github.com/ros2/ros2cli/pull/638>`_.

    To dump parameters to a file, run:

    .. code-block:: bash

      ros2 param dump /my_node_name > my_node_name.yaml

robot_state_publisher
^^^^^^^^^^^^^^^^^^^^^

Removal of deprecated ``use_tf_static`` parameter
"""""""""""""""""""""""""""""""""""""""""""""""""

The deprecated ``use_tf_static`` parameter has been removed from ``robot_state_publisher``.
This means that static transforms are unconditionally published to the ``/tf_static`` topic, and that the static transforms are published in a ``transient_local`` Quality of Service.
This was the default behavior, and the behavior which the ``tf2_ros::TransformListener`` class expected before, so most code will not have to be changed.
Any code that was relying on ``robot_state_publisher`` to periodically publish static transforms to ``/tf`` will have to be updated to subscribe to ``/tf_static`` as a ``transient_local`` subscription instead.


rosidl_cmake
^^^^^^^^^^^^

Deprecation of ``rosidl_target_interfaces()``
"""""""""""""""""""""""""""""""""""""""""""""

The CMake function ``rosidl_target_interfaces()`` has been deprecated, and now issues a CMake warning when called.
Users wanting to use messages/services/actions in the same ROS package that generated them should instead call ``rosidl_get_typesupport_target()`` and then ``target_link_libraries()`` to make their targets depend on the returned typesupport target.
See https://github.com/ros2/rosidl/pull/606 for more details, and https://github.com/ros2/demos/pull/529 for an example of using the new function.

geometry2
^^^^^^^^^

Deprecation of TF2Error::NO_ERROR, etc
""""""""""""""""""""""""""""""""""""""

The ``tf2`` library uses an enumeration called ``TF2Error`` to return errors.
Unfortunately, one of the enumerators in there is called ``NO_ERROR``, which conflicts with a macro on Windows.
To remedy this, a new set of enumerators in ``TF2Error`` were created, each with a ``TF2`` prefix.
The previous enumerators are still available, but are now deprecated and will print a deprecation warning if used.
All code that uses the ``TF2Error`` enumerator should be updated to use the new ``TF2`` prefixed errors.
See https://github.com/ros2/geometry2/pull/349 for more details.


Known Issues
------------

To come.

Release Timeline
----------------

    Mon. March 21, 2022 - Alpha + RMW freeze
        Preliminary testing and stabilization of ROS Base [1]_ packages, and API and feature freeze for RMW provider packages.

    Mon. April 4, 2022 - Freeze
        API and feature freeze for ROS Base [1]_ packages in Rolling Ridley.
        Only bug fix releases should be made after this point.
        New packages can be released independently.

    Mon. April 18, 2022 - Branch
        Branch from Rolling Ridley.
        ``rosdistro`` is reopened for Rolling PRs for ROS Base [1]_ packages.
        Humble development shifts from ``ros-rolling-*`` packages to ``ros-humble-*`` packages.

    Mon. April 25, 2022 - Beta
        Updated releases of ROS Desktop [2]_ packages available.
        Call for general testing.

    Mon. May 16, 2022 - Release Candidate
        Release Candidate packages are built.
        Updated releases of ROS Desktop [2]_ packages available.

    Thu. May 19, 2022 - Distro Freeze
        Freeze rosdistro.
        No PRs for Humble on the ``rosdistro`` repo will be merged (reopens after the release announcement).

    Mon. May 23, 2022 - General Availability
        Release announcement.
        ``rosdistro`` is reopened for Humble PRs.

.. [1] The ``ros_base`` variant is described in `REP 2001 (ros-base) <https://www.ros.org/reps/rep-2001.html#ros-base>`_.
.. [2] The ``desktop`` variant is described in `REP 2001 (desktop-variants) <https://www.ros.org/reps/rep-2001.html#desktop-variants>`_.
