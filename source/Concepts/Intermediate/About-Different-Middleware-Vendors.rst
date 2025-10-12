.. redirect-from::

    DDS-and-ROS-middleware-implementations
    Concepts/About-Different-Middleware-Vendors

Different ROS 2 middleware vendors
==================================

.. contents:: Table of Contents
   :local:

ROS 2 supports multiple middleware implementations which provide discovery, serialisation, and transportation communications.
This flexibility exists because it is not necessarily "one size fits all" when it comes to choosing a middleware vendor/implementation.
While primarily built on top of DDS/RTPS as its initial middleware foundation, the ROS 2 ecosystem has expanded to include other middleware architectures such as Zenoh.

DDS middleware
--------------

`DDS <https://www.omg.org/omg-dds-portal>`__ is an industry standard which is implemented by a range of vendors, such as RTI's `Connext DDS <https://www.rti.com/products/>`__, eProsima's `Fast DDS <https://fast-dds.docs.eprosima.com/>`__, Eclipse's `Cyclone DDS <https://projects.eclipse.org/projects/iot.cyclonedds>`__, or GurumNetworks's `GurumDDS <https://gurum.cc/index_eng>`__.
RTPS (a.k.a. `DDSI-RTPS <https://www.omg.org/spec/DDSI-RTPS/About-DDSI-RTPS/>`__\ ) is the wire protocol used by DDS to communicate over the network.

`This article <https://design.ros2.org/articles/ros_on_dds.html>`__ explains the motivation behind using DDS implementations, and/or the RTPS wire protocol of DDS, in detail.
In summary, DDS is an end-to-end middleware that provides features which are relevant to ROS systems, such as distributed discovery (not centralized like in ROS 1) and control over different "Quality of Service" (a.k.a. QoS) options for the transportation.

Zenoh middleware
----------------

`Zenoh <https://docs.ros.org/en/rolling/Installation/RMW-Implementations/Non-DDS-Implementations/Working-with-Zenoh.html>`_ is a protocol that integrates internet-scale publish/subscribe with distributed querying.
It is designed for efficient communication in a wide range of deployments that vary from server-grade hardware and networks to resource-constrained edge devices.
Zenoh extends the privilege of location transparency to storage data, allowing queries to be addressed with no concerns about where the data is stored.

As an RMW implementation for ROS 2, Zenoh offers a more lightweight alternative to DDS and maintains Quality of Service features (in Zenoh, there are essentially no "incompatible" QoS settings).
Its minimal wire overhead and flexible routing make Zenoh well-suited for challenging network conditions.

Supported RMW implementations
-----------------------------

.. list-table::
   :header-rows: 1

   * - Product name
     - License
     - RMW implementation
     - Status
   * - eProsima *Fast DDS*
     - Apache 2
     - ``rmw_fastrtps_cpp``
     - Full support.
       Default RMW.
       Packaged with binary releases.
   * - Eclipse *Cyclone DDS*
     - Eclipse Public License v2.0
     - ``rmw_cyclonedds_cpp``
     - Full support.
       Packaged with binary releases.
   * - RTI *Connext DDS*
     - Commercial, research
     - ``rmw_connextdds``
     - Full support.
       Support included in binaries, but Connext installed separately.
   * - GurumNetworks *GurumDDS*
     - Commercial
     - ``rmw_gurumdds_cpp``
     - Community support.
       Support included in binaries, but GurumDDS installed separately.
   * - Eclipse *Zenoh*
     - Eclipse Public License v2.0
     - ``rmw_zenoh_cpp``
     - Full support.
       Packaged with binary releases starting with Kilted Kaiju.

For practical information on working with multiple RMW implementations, see the :doc:`"Working with multiple RMW implementations" <../../How-To-Guides/Working-with-multiple-RMW-implementations>` tutorial.

Choosing a middleware implementation
------------------------------------

There are many factors you might consider while choosing a middleware implementation: logistical considerations like the license, or technical considerations like platform availability, resource utilisation, or computation footprint.
Vendors may provide more than one middleware implementation targeted at meeting different needs.

For example, RTI has a few variations of their Connext implementation that vary in purpose, like one that specifically targets microcontrollers and another which targets applications requiring special safety certifications (we only support their standard desktop version at this time).
Eclipse offers both Cyclone DDS and Zenoh.
Cyclone DDS is one of the lighter DDS implementations, optimised for real-time deterministic communication, while Zenoh is designed for IoT and edge computing scenarios where high throughput, low latency, and interoperability across heterogeneous environments are primary concerns.

In order to use a middleware implementation with ROS 2, a "\ **R**\ OS **M**\ iddle\ **w**\ are interface" (a.k.a. ``rmw`` interface or just ``rmw``\ ) package needs to be created that implements the abstract ROS middleware interface using that specific implementation's API and tools.
It's a lot of work to implement and maintain RMW packages for supporting various middleware implementations, but this diversity is important for ensuring that the ROS 2 codebase is not tied to any one particular implementation, as users may wish to switch out implementations depending on their project's needs.

Multiple RMW implementations
----------------------------

The ROS 2 binary releases for currently active distros have built-in support for several RMW implementations out of the box (Fast DDS, RTI Connext Pro, Eclipse Cyclone DDS, GurumNetworks GurumDDS).
Beginning with ROS 2 Kilted Kaiju, this also includes Eclipse Zenoh.
The default is Fast DDS, which works without any additional installation steps because we distribute it with our binary packages.

RMWs other than Fast DDS, like Cyclone DDS, Connext or GurumDDS can be enabled by :doc:`installing additional packages <../../Installation/RMW-Implementations>`, and without having to rebuild anything or replace any existing packages.

A ROS 2 workspace that has been built from source may build and install multiple RMW implementations simultaneously.
While the core ROS 2 code is being compiled, any RMW implementation that is found will be built if the relevant middleware implementation has been installed properly and the relevant environment variables have been configured.
For example, if the code for the `RMW package for RTI Connext DDS <https://github.com/ros2/rmw_connextdds>`__ is in the workspace, it will be built if an installation of RTI's Connext Pro can also be found.

If a ROS 2 workspace has multiple RMW implementations, Fast DDS is selected as the default RMW implementation if it is available.
If Fast DDS is not installed, the default middleware will be selected based on alphabetical order of the package identifiers.
The implementation identifier is the name of the ROS package that provides the RMW implementation, e.g. ``rmw_cyclonedds_cpp``.
For example, if both ``rmw_cyclonedds_cpp`` and ``rmw_connextdds`` ROS packages are installed, ``rmw_connextdds`` would be the default.
If ``rmw_fastrtps_cpp`` is ever installed, it would be the default.

See the :doc:`guide <../../How-To-Guides/Working-with-multiple-RMW-implementations>` for how to specify which RMW implementation is to be used when running the ROS 2 examples.

.. _different-middleware-vendors-cross-vendor-communication:

Cross-Vendor Communication among DDS middleware
-----------------------------------------------

For many cases you will find that nodes using different DDS middleware implementations are able to communicate, however this is not true under all circumstances.
While the different DDS implementations may be compatible in limited circumstances, this is not guaranteed.
Thus it is suggested that users ensure that all parts of a distributed system are using the same ROS version and the same RMW implementation.
