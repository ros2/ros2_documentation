.. meta::
   :contentType: TBD
   :experience: TBD
   :area: installation
   :distribution: {DISTRO}
   :product: {PRODUCT}

.. redirect-from::

    Installation/_Apt-Upgrade-Admonition

Upgrading with apt
==================

.. short-description::
   Keeping Ubuntu packages up to date helps prevent dependency issues before installing ROS packages with apt.
   This article explains why you should upgrade your system first and shows the command used to update installed packages.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Contents
   :depth: 2
   :local:

ROS 2 packages are built on frequently updated Ubuntu systems.
It is always recommended that you ensure your system is up to date before installing new packages.

.. code-block:: console

   $ sudo apt upgrade
