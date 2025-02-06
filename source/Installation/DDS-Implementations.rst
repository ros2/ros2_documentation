DDS implementations
===================

By default, ROS 2 uses DDS as its `middleware <https://design.ros2.org/articles/ros_on_dds.html>`__.
It is compatible with multiple DDS or RTPS (the DDS wire protocol) vendors.
There is currently support for eProsima's Fast DDS, RTI's Connext DDS, Eclipse Cyclone DDS, and GurumNetworks GurumDDS.
See https://ros.org/reps/rep-2000.html for supported DDS vendors by distribution.

The default DDS vendor is eProsima's Fast DDS.

* :doc:`Working with Eclipse Cyclone DDS <DDS-Implementations/Working-with-Eclipse-CycloneDDS>` explains how to utilize Cyclone DDS.
* :doc:`Working with eProsima Fast DDS <DDS-Implementations/Working-with-eProsima-Fast-DDS>` explains how to utilize Fast DDS.
* :doc:`Working with RTI Connext DDS <DDS-Implementations/Install-Connext-University-Eval>` explains how to utilize and evaluate RTI Connext DDS.
* :doc:`Working with GurumNetworks GurumDDS <DDS-Implementations/Working-with-GurumNetworks-GurumDDS>` explains how to utilize GurumDDS.

.. toctree::
   :hidden:
   :glob:

   DDS-Implementations/*

If you would like to use one of the other vendors you will need to install their software separately before building.
The ROS 2 build will automatically build support for vendors that have been installed and sourced correctly.

Once you've installed a new DDS vendor, you can change the vendor used at runtime: :doc:`Working with Multiple RMW Implementations <../How-To-Guides/Working-with-multiple-RMW-implementations>`.
