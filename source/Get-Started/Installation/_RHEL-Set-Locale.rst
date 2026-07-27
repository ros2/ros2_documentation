.. meta::
   :contentType: TBD
   :experience: TBD
   :area: installation
   :distribution: {DISTRO}
   :product: {PRODUCT}

.. redirect-from::

    Installation/_RHEL-Set-Locale

Setting locale on RHEL
======================

.. short-description::
   ROS requires a locale that supports UTF-8 so command-line tools and package installation steps handle text consistently on RHEL.
This article explains how to check the current locale, install the required language packages, and set LANG to a supported UTF-8 value.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Contents
   :depth: 2
   :local:

Make sure you have a locale which supports ``UTF-8``.
If you are in a minimal environment (such as a docker container), the locale may be something minimal like ``C``.
We test with the following settings.
However, it should be fine if you're using a different UTF-8 supported locale.

.. code-block:: console

   $ locale  # check for UTF-8

   $ sudo dnf install langpacks-en glibc-langpack-en
   $ export LANG=en_US.UTF-8

   $ locale  # verify settings
