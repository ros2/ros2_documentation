.. redirect-from::

    How-To-Guides/Releasing/_Next-Steps

.. meta::
   :contentType: how-to
   :experience: intermediate
   :area: builds, tools
   :distribution: {DISTRO}
   :product: {PRODUCT}

Releasing next steps - how-to
=============================

.. short-description::
   After a ROS release pull request is submitted, it must be reviewed, built, tested, and synchronized before it reaches users.
   This article describes what happens after submission and where to follow release sync updates.
   After reading it, you will understand when packages become available for testing and general use.

.. showmeta::
    :order: area, contentType, experience
    :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Table of Contents
   :depth: 2
   :local:

Summary
-------

After you submit the release pull request, a ``rosdistro`` maintainer usually reviews and merges it within one or two days.
If the package build succeeds, the packages normally become available in ``ros-testing`` within 24-48 hours.
The distribution release manager manually synchronizes ros-testing into the main ROS repository approximately every two to four weeks.

Once your pull request has been submitted, usually within one or two days, one of the maintainers of rosdistro will review and merge your Pull Request.
If your package build is successful, in 24-48 hours your packages will become available in the **ros-testing** repository, where you can :doc:`test your pre-release binaries <../../Testing/Testing>`.

Approximately every two to four weeks, the distribution's release manager manually synchronizes the contents of ros-testing into the main ROS repository.
This is when your packages actually become available to the rest of the ROS community.
To get updates on when the next synchronization (sync) is coming, subscribe to the `Packaging and Release Management Category on Open Robotics Discourse <https://discourse.openrobotics.org/c/ros/release/16>`_.
