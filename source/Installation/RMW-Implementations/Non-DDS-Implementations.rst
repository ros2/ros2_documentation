Non-DDS-Implementations
=======================

* :doc:`Working with Zenoh <Non-DDS-Implementations/Working-with-Zenoh>` explains how to utilize Zenoh.

.. toctree::
   :hidden:
   :glob:

   Non-DDS-Implementations/*

If you would like to use one of the other vendors you will need to install their software separately before building.
The ROS 2 build will automatically build support for vendors that have been installed and sourced correctly.

Once you've installed a new RMW vendor, you can change the vendor used at runtime: :doc:`Working with Multiple RMW Implementations <../../How-To-Guides/Working-with-multiple-RMW-implementations>`.
