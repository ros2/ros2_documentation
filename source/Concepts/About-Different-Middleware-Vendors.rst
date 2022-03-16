.. redirect-from::

    DDS-and-ROS-middleware-implementations

About different ROS 2 DDS/RTPS vendors
======================================

ROS 2 is built on top of DDS/RTPS as its middleware, which provides discovery, serialization and transportation.
`This article <https://design.ros2.org/articles/ros_on_dds.html>`__ explains the motivation behind using DDS implementations, and/or the RTPS wire protocol of DDS, in detail.
In summary, DDS is an end-to-end middleware that provides features which are relevant to ROS systems, such as distributed discovery (not centralized like in ROS 1) and control over different "Quality of Service" options for the transportation.

`DDS <https://www.omg.org/omg-dds-portal>`__ is an industry standard which is implemented by a range of vendors, such as RTI's `Connext DDS <https://www.rti.com/products/>`__, eProsima's `Fast DDS <https://fast-dds.docs.eprosima.com/>`__, Eclipse's `Cyclone DDS <https://projects.eclipse.org/projects/iot.cyclonedds>`__, or GurumNetworks's `GurumDDS <https://gurum.cc/index_eng>`__.
RTPS (a.k.a. `DDSI-RTPS <https://www.omg.org/spec/DDSI-RTPS/About-DDSI-RTPS/>`__\ ) is the wire protocol used by DDS to communicate over the network.

ROS 2 supports multiple DDS/RTPS implementations because it is not necessarily "one size fits all" when it comes to choosing a vendor/implementation.
There are many factors you might consider while choosing a middleware implementation: logistical considerations like the license, or technical considerations like platform availability, or computation footprint.
Vendors may provide more than one DDS or RTPS implementation targeted at meeting different needs.
For example, RTI has a few variations of their Connext implementation that vary in purpose, like one that specifically targets microcontrollers and another which targets applications requiring special safety certifications (we only support their standard desktop version at this time).

In order to use a DDS/RTPS implementation with ROS 2, a "\ **R**\ OS **M**\ iddle\ **w**\ are interface" (a.k.a. ``rmw`` interface or just ``rmw``\ ) package needs to be created that implements the abstract ROS middleware interface using the DDS or RTPS implementation's API and tools.
It's a lot of work to implement and maintain RMW packages for supporting DDS implementations, but supporting at least a few implementations is important for ensuring that the ROS 2 codebase is not tied to any one particular implementation, as users may wish to switch out implementations depending on their project's needs.

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
     - Full support. Default RMW. Packaged with binary releases.
   * - Eclipse *Cyclone DDS*
     - Eclipse Public License v2.0
     - ``rmw_cyclonedds_cpp``
     - Full support. Packaged with binary releases.
   * - RTI *Connext DDS*
     - commercial, research
     - ``rmw_connextdds``
     - Full support. Support included in binaries, but Connext installed separately.
   * - GurumNetworks *GurumDDS*
     - commercial
     - ``rmw_gurumdds_cpp``
     - Community support. Support included in binaries, but GurumDDS installed separately.

For practical information on working with multiple RMW implementations, see the :doc:`"Working with multiple RMW implementations" <../How-To-Guides/Working-with-multiple-RMW-implementations>` tutorial.

Multiple RMW implementations
----------------------------

The ROS 2 binary releases for currently active distros have built-in support for several RMW implementations out of the box (Fast DDS, RTI Connext Pro, Eclipse Cyclone DDS, GurumNetworks GurumDDS).
The default is Fast DDS, which works without any additional installation steps because we distribute it with our binary packages.

Other RMWs like Cyclone DDS, Connext or GurumDDS can be enabled by :doc:`installing additional packages <../Installation/DDS-Implementations>`, but without having to rebuild anything or replace any existing packages.

A ROS 2 workspace that has been built from source may build and install multiple RMW implementations simultaneously.
While the core ROS 2 code is being compiled, any RMW implementation that is found will be built if the relevant DDS/RTPS implementation has been installed properly and the relevant environment variables have been configured.
For example, if the code for the `RMW package for RTI Connext DDS <https://github.com/ros2/rmw_connextdds>`__ is in the workspace, it will be built if an installation of RTI's Connext Pro can also be found.

For many cases you will find that nodes using different RMW implementations are able to communicate, however this is not true under all circumstances.
Here is a list of inter-vendor communication configurations that are not supported:

- Fast DDS <-> Connext
   - ``WString`` published by Fast DDS can't be received correctly by Connext on macOS
- Connext <-> Cyclone DDS
   - does not support pub/sub communication for ``WString``

Default RMW implementation
--------------------------

If a ROS 2 workspace has multiple RMW implementations, Fast DDS is selected as the default RMW implementation if it is available.
If the Fast DDS RMW implementation is not installed, the RMW implementation with the first RMW implementation identifier in alphabetical order will be used.
The implementation identifier is the name of the ROS package that provides the RMW implementation, e.g. ``rmw_cyclonedds_cpp``.
For example, if both ``rmw_cyclonedds_cpp`` and ``rmw_connextdds`` ROS packages are installed, ``rmw_connextdds`` would be the default.
If ``rmw_fastrtps_cpp`` is ever installed, it would be the default.

See the :doc:`guide <../How-To-Guides/Working-with-multiple-RMW-implementations>` for how to specify which RMW implementation is to be used when running the ROS 2 examples.
