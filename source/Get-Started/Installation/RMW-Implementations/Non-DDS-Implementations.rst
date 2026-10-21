.. meta::
   :contentType: about
   :experience: intermediate, expert
   :area: installation
   :distribution: {DISTRO}
   :product: {PRODUCT}

.. redirect-from::

    Installation/RMW-Implementations/Non-DDS-Implementations

Non-DDS-Implementations
=======================

.. short-description::
   Non-Data-Distribution-Service (non-DDS) implementations provide alternative middleware options for ROS communication when a non-DDS vendor better suits your system requirements.
   This article links to supported non-DDS guidance and explains how installed RMW vendors can be selected at runtime.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Contents
   :depth: 2
   :local:

* :doc:`Working with Zenoh <Non-DDS-Implementations/Working-with-Zenoh>` explains how to utilize Zenoh.

.. toctree::
   :hidden:
   :glob:

   Non-DDS-Implementations/*

If you would like to use one of the other vendors you will need to install their software separately before building.
The ROS 2 build will automatically build support for vendors that have been installed and sourced correctly.

Once you've installed a new RMW vendor, you can change the vendor used at runtime: :doc:`Working with Multiple RMW Implementations <Working-with-multiple-RMW-implementations>`.
