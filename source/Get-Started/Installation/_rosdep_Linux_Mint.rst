.. meta::
   :contentType: TBD
   :experience: TBD
   :area: installation
   :distribution: {DISTRO}
   :product: {PRODUCT}

.. redirect-from::

    Installation/_rosdep_Linux_Mint

Using rosdep on Linux Mint
==========================

.. short-description::
   rosdep helps install the system dependencies needed to build and run ROS packages, but Linux Mint may require extra OS identification because it is based on Ubuntu.
   This article explains how to use rosdep on Linux Mint by appending the correct Ubuntu OS override when the default command reports an unsupported OS.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Contents
   :depth: 2
   :local:

**Note**: If you're using a distribution that is based on Ubuntu (like Linux Mint) but does not identify itself as such, you'll get an error message like ``Unsupported OS [mint]``.
In this case append ``--os=ubuntu:resolute`` to the above command.
