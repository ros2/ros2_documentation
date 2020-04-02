.. redirect-from::

    DDS-and-ROS-middleware-implementations

ROS 2 and different DDS/RTPS vendors
====================================

ROS 2 is built on top of DDS/RTPS as its middleware, which provides discovery, serialization and transportation.
`This article <http://design.ros2.org/articles/ros_on_dds.html>`__ explains the motivation behind using DDS implementations, and/or the RTPS wire protocol of DDS, in detail, but in summary DDS is an end-to-end middleware that provides features which are relevant to ROS systems, such as distributed discovery (not centralized like in ROS 1) and control over different "Quality of Service" options for the transportation.

`DDS <http://portals.omg.org/dds/>`__ is an industry standard which is then implemented by a range of organizations and vendors, such as Eclipse IoT open source project `Eclipse Cyclone DDS <https://github.com/eclipse-cyclonedds/cyclonedds>`__ or RTI's implementation `Connext <https://www.rti.com/products/>`__.  `DDSI-RTPS <https://www.omg.org/spec/DDSI-RTPS/About-DDSI-RTPS/>`__\  is the wire protocol used by DDS to communicate over the network, and there are implementations which do not fulfill the full DDS API, but provide sufficient functionality for ROS 2, such as eProsima's implementation `Fast RTPS <http://www.eprosima.com/index.php/products-all/eprosima-fast-rtps>`__.

ROS 2 supports multiple DDS/RTPS implementations because it is not necessarily "one size fits all" when it comes to choosing a vendor/implementation.
There are many factors you might consider while choosing a middleware implementation: logistical considerations like the license, or technical considerations like ease, performance, platform availability, or computation footprint.

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
   * - eProsima *Fast RTPS*
     - Apache 2
     - ``rmw_fastrtps_cpp``
     - Full support. Default RMW. Packaged with binary releases.
   * - *Eclipse Cyclone DDS* a project of Eclipse IoT and OpenADx WG
     - Eclipse Public License v. 2.0
     - ``rmw_cyclonedds_cpp``
     - Full support. Tier-1 RMW in Foxy. Packaged with binary releases.
   * - RTI *Connext*
     - commercial, research
     - ``rmw_connext_cpp``
     - Full support. Support included in binaries, but Connext installed separately.
   * - RTI *Connext* (dynamic implementation)
     - commercial, research
     - ``rmw_connext_dynamic_cpp``
     - Support paused. Full support until alpha 8.*
   * - ADLINK *Opensplice DDS*
     - Apache 2, commercial
     - ``rmw_opensplice_cpp``
     - Partial support. Support included in binaries until Foxy. OpenSplice is installed separately.


*"Partial support" means that one or more of the features required by the rmw interface is not implemented.*

For practical information on working with multiple RMW implementations, see the `"Working with multiple RMW implementations" <../Tutorials/Working-with-multiple-RMW-implementations>` tutorial.
