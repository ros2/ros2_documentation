DDS implementations
===================

These are the available DDS implementations:

* :doc:`Working with Eclipse Cyclone DDS <DDS-Implementations/Working-with-Eclipse-CycloneDDS>` explains how to utilize Cyclone DDS.
* :doc:`Working with eProsima Fast DDS <DDS-Implementations/Working-with-eProsima-Fast-DDS>` explains how to utilize Fast DDS.
* :doc:`Working with RTI Connext DDS <DDS-Implementations/Working-with-RTI-Connext-DDS>` explains how to utilize RTI Connext DDS.
* :doc:`Working with GurumNetworks GurumDDS <DDS-Implementations/Working-with-GurumNetworks-GurumDDS>` explains how to utilize GurumDDS.

.. toctree::
   :hidden:
   :glob:

   DDS-Implementations/*

If you would like to use one of the other vendors you will need to install their software separately before building.
The ROS 2 build will automatically build support for vendors that have been installed and sourced correctly.

Once you've installed a new RMW vendor, you can change the vendor used at runtime: :doc:`Working with Multiple RMW Implementations <../../How-To-Guides/Working-with-multiple-RMW-implementations>`.
