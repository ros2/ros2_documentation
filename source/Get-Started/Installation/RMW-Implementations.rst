.. meta::
   :contentType: about
   :experience: intermediate, expert
   :area: installation
   :distribution: {DISTRO}
   :product: {PRODUCT}

.. redirect-from::

    Installation/DDS-Implementations
    Installation/RMW-Implementations

RMW implementations
===================

.. short-description::
   ROS can use different middleware vendors to support communication between nodes, depending on your platform, requirements, and distribution.
   This article introduces RMW implementations, identifies the default vendor, and links to guidance for DDS and non-DDS middleware options.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Contents
   :depth: 2
   :local:

By default, ROS 2 uses DDS as its `middleware <https://design.ros2.org/articles/ros_on_dds.html>`__.
It is compatible with multiple DDS or RTPS (the DDS wire protocol) vendors.
There is currently support for eProsima's Fast DDS, RTI's Connext DDS, Eclipse Cyclone DDS, and GurumNetworks GurumDDS.

It also supports non DDS RMW implementations such as Zenoh.

See `REP-2000 <https://reps.openrobotics.org/rep-2000/>`__ for supported RMW vendors by distribution.

The default RMW vendor is eProsima's Fast DDS.

Review all the possible options:

.. toctree::
   :hidden:
   :glob:

   RMW-Implementations/*

* :doc:`DDS implementations <RMW-Implementations/DDS-Implementations>` explains how to use DDS.
* :doc:`Non DDS implementations <RMW-Implementations/Non-DDS-Implementations>` explains how to use non DDS implementations.
