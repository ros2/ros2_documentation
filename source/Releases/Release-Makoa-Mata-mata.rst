.. _upcoming-release:

.. _makoa-release:

Makoa Mata-mata (codename ``makoa``; May, 2027)
===============================================

.. toctree::
   :hidden:

   makoa/release-timeline.rst
   makoa/supported-platforms.rst

*Makoa Mata-mata* is the thirtienth release of ROS 2.
It is a regular release, and is supported until December 2028.

* TODO - link installation docs
* :doc:`makoa/release-timeline`
* :doc:`makoa/supported-platforms`

New Features in Makoa
---------------------

TODO

Changes since the Lyrical release
---------------------------------

Removed RMW for Fast DDS with dynamic typesupport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Package ``rmw_fastrtps_dynamic_cpp`` has been removed.
This means that users will no longer be able to select ``RMW_IMPLEMENTATION=rmw_fastrtps_dynamic_cpp``.
Users are encouraged to use ``RMW_IMPLEMENTATION=rmw_fastrtps_cpp`` instead.
