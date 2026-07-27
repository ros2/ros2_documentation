.. meta::
   :contentType: TBD
   :experience: TBD
   :area: installation
   :distribution: {DISTRO}
   :product: {PRODUCT}

.. redirect-from::

    Installation/_Ubuntu-Set-Locale

Setting locale on Ubuntu
========================

.. short-description::
   ROS requires a locale that supports ``UTF-8`` so command-line tools and package installation steps handle text consistently on Ubuntu.
   This article explains how to check the current locale, install and generate the required locale packages, and set ``LANG`` to a supported ``UTF-8`` value.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Contents
   :depth: 2
   :local:

Make sure you have a locale which supports ``UTF-8``.
If you are in a minimal environment (such as a docker container), the locale may be something minimal like ``POSIX``.
We test with the following settings.
However, it should be fine if you're using a different UTF-8 supported locale.

.. code-block:: console

   $ locale  # check for UTF-8

   $ sudo apt update && sudo apt install locales
   $ sudo locale-gen en_US en_US.UTF-8
   $ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   $ export LANG=en_US.UTF-8

   $ locale  # verify settings
