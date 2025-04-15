RMW implementations
===================

By default, ROS 2 uses DDS as its `middleware <https://design.ros2.org/articles/ros_on_dds.html>`__.
It is compatible with multiple DDS or RTPS (the DDS wire protocol) vendors.
There is currently support for eProsima's Fast DDS, RTI's Connext DDS, Eclipse Cyclone DDS, and GurumNetworks GurumDDS.
See `REP-2000 <https://ros.org/reps/rep-2000.html>`__ for supported DDS vendors by distribution.

The default DDS vendor is eProsima's Fast DDS.

Review all the possible options:

.. toctree::
   :hidden:
   :glob:

   RMW-Implementations/*


* :doc:`Working DDS <RMW-Implementations/DDS-Implementations>` explains how to use DDS.
* :doc:`Working non DDS <RMW-Implementations/Non-DDS-Implementations>` explains how to use non DDS implementations.
