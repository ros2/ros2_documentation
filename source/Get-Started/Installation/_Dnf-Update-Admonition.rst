.. meta::
   :contentType: TBD
   :experience: TBD
   :area: installation
   :distribution: {DISTRO}
   :product: {PRODUCT}

.. redirect-from::

    Installation/_Dnf-Update-Admonition

Updating with dnf
=================

.. short-description::
   RHEL package installations work best when the operating system is up to date.
   This article describes the recommended dnf update command to run before installing ROS packages.
   After you follow these steps, your system will be ready to install packages against current RHEL dependencies.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Contents
   :depth: 2
   :local:

ROS 2 packages are built on frequently updated RHEL systems.
It is always recommended that you ensure your system is up to date before installing new packages.

.. code-block:: console

   $ sudo dnf update
