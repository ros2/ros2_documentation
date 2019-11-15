
ROS 2 Eloquent Elusor (codename 'eloquent'; November 22nd, 2019)
================================================================

.. contents:: Table of Contents
   :depth: 2
   :local:

*Eloquent Elusor* is the fifth release of ROS 2.

Supported Platforms
-------------------

Eloquent Elusor is primarily supported on the following platforms:

Tier 1 platforms:

* Ubuntu 18.04 (Bionic): ``amd64`` and ``arm64``
* Mac OS X 10.12 (Sierra)
* Windows 10 (Visual Studio 2019)

Tier 2 platforms:

* Ubuntu 18.04 (Bionic): ``arm32``

Tier 3 platforms:

* Debian Stretch (9): ``amd64``, ``arm64`` and ``arm32``
* OpenEmbedded Thud (2.6) / webOS OSE: ``arm32`` and ``x86``

For more information about RMW implementations, compiler / interpreter versions, and system dependency versions see `REP 2000 <http://www.ros.org/reps/rep-2000.html>`__.


New features in this ROS 2 release
----------------------------------

During the development the `Eloquent meta ticket <https://github.com/ros2/ros2/issues/734>`__ on GitHub contains an up-to-date state of the ongoing high level tasks as well as references specific tickets with more details.

Changes since the Dashing release
---------------------------------

geometry_msgs
^^^^^^^^^^^^^

The ``geometry_msgs/msg/Quaternion.msg`` interface now default initializes to a valid quaternion, with the following values:

.. math::

    x = 0 \\
    y = 0 \\
    z = 0 \\
    w = 1

Here is the pull request for more detail: `https://github.com/ros2/common_interfaces/pull/74 <https://github.com/ros2/common_interfaces/pull/74>`_

Static transform broadcasters and listeners now use QoS durability ``transient_local``` on the ``/tf_static`` topic.
Similar to the latched setting in ROS 1, static transforms only need to be published once.
New listeners will receive transforms from all static broadcasters that are alive and have published before.
See this pull request for more detail: `https://github.com/ros2/geometry2/pull/160 <https://github.com/ros2/geometry2/pull/160>`_

rclcpp
^^^^^^

API Break with ``get_actual_qos()``
"""""""""""""""""""""""""""""""""""

Introduced in Dashing, the ``get_actual_qos()`` method on the ``PublisherBase`` and ``SubscriptionBase`` previously returned an rmw type, ``rmw_qos_profile_t``, but that made it awkward to reuse with the creation of other entities.
Therefore it was updated to return a ``rclcpp::QoS`` instead.

Existing code will need to use the ``rclcpp::QoS::get_rmw_qos_profile()`` method if the rmw profile is still required.
For example:

.. code-block:: cpp

    void my_func(const rmw_qos_profile_t & rmw_qos);

    /* Previously: */
    // my_func(some_pub->get_actual_qos());
    /* Now: */
    my_func(some_pub->get_actual_qos()->get_rmw_qos_profile());

The rationale for breaking this directly rather than doing a tick-tock is that it is a new function and is expected to be used infrequently by users.
Also, since only the return type is changing, adding a new function with a different would be to only way to do a deprecation cycle and ``get_actual_qos()`` is the most appropriate name, so we would be forced to pick a less obvious name for the method.

API Break with Publisher and Subscription Classes
"""""""""""""""""""""""""""""""""""""""""""""""""

In an effort to streamline the construction of Publishers and Subscriptions, the API of the constructors were changed.

It would be impossible to support a deprecation cycle, because the old signature takes an rcl type and the new one takes the ``NodeBaseInterface`` type so that it can get additional information it now needs, and there's no way to get the additional information needed from just the rcl type.
The new signature could possibly be backported if that would help contributors, but since the publishers and subscriptions are almost always created using the factory functions or some other higher level API, we do not expect this to be a problem for most users.

Please see the original pr for more detail and comment there if this causes issues:

`https://github.com/ros2/rclcpp/pull/867 <https://github.com/ros2/rclcpp/pull/867>`_

rmw
^^^

API Break Due to Addition of Publisher and Subscription Options
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

The ``rmw_create_publisher()`` method had a new argument added of type ``const rmw_publisher_options_t *``.
This new structure holds options (beyond the typesupport, topic name, and QoS) for new publishers.

The ``rmw_create_subscription()`` method had one argument removed, ``bool ignore_local_publications``, and replaced by the new options of type ``const rmw_subscription_options_t *``.
The ``ignore_local_publications`` option was moved into the new ``rmw_subscription_options_t`` type.

In both cases the new argument, which are pointers, may never be null, and so the rmw implementations should check to make sure the options are not null.
Additionally, the options should be copied into the corresponding rmw structure.

See this pull request, and the associated pull requests for more details:

`https://github.com/ros2/rmw/pull/187 <https://github.com/ros2/rmw/pull/187>`_

Timeline before the release
---------------------------

A few milestones leading up to the release:

    Mon. Sep 30th (alpha)
        First releases of core packages available.
        Testing can happen from now on (some features might not have landed yet).

    Fri. Oct 18th
        API and feature freeze for core packages
        Only bug fix releases should be made after this point.
        New packages can be released independently.

    Thu. Oct 24th (beta)
        Updated releases of core packages available.
        Additional testing of the latest features.

    Wed. Nov 13th (release candidate)
        Updated releases of core packages available.

    Tue. Nov 19th
        Freeze rosdistro.
        No PRs for Eloquent on the `rosdistro` repo will be merged (reopens after the release announcement).

