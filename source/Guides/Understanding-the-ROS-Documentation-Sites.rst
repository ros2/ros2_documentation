Understanding the ROS Documentation Sites
=========================================

.. contents:: Table of Contents
   :depth: 2
   :local:

This guide explains the ROS 1 and ROS 2 documentation infrastructure.
It should be helpful in understanding where specific resources live, how to ask questions, and which sites are being maintained.

`ROS Answers <https://answers.ros.org/questions/>`_
---------------------------------------------------

:Purpose:
  A Q&A community website for ROS 1 and 2 that is similar to `Stack Exchange <https://stackexchange.com/>`_.

`ROS Design <http://design.ros2.org/>`_
---------------------------------------

:Purpose:
  A place that documents many of the early ROS 2 design decisions.

:Notes:
  :ref:`REPs Section` are preferable to new documents in ROS Design.

`ROS Discourse <https://discourse.ros.org/>`_
---------------------------------------------

:Purpose:
  A forum for general discussions and announcements for the ROS community.

`ROS Docs <https://docs.ros.org>`_ (this site)
----------------------------------------------

:Purpose:

  * ROS 1 and ROS 2 API documentation
  * ROS 2 core documentation, such as installation instructions, tutorials, guides, roadmap, etc.

`ROS Enhancement Proposals (REPs) <https://ros.org/reps/rep-0000.html>`_
------------------------------------------------------------------------

:Purpose:
  A place for ROS 1 and ROS 2 design and conventions.

:Notes:
  REPs are preferred to ROS Design, since they have a more established review process.

`ROS Index <https://index.ros.org/>`_
-------------------------------------

:Purpose:
  An indexed list of all packages that links to additional information.

:Notes:
  ROS Index is useful for

  * Seeing which ROS distributions a package supports (ROS 1 and ROS 2)
  * Linking to a packages repository, API documentation, or website
  * Inspecting a package's license, build type, maintainers, status, and dependencies
  * Finding questions on `ROS Answers <https://answers.ros.org/questions/>`_ that involve the package

  You can think of ROS 2 index as something similar to the `Python Package Index (PyPi) <https://pypi.org/>`_ for ROS packages.

`ROS Prerelease <http://prerelease.ros.org/>`_
----------------------------------------------

:Purpose:
  A website that helps you generate commands to emulate the `ROS Buildfarm <https://build.ros.org/>`_ on your local machine.

:Notes:
  Currently, the frontend of this site only shows ROS 1 distributions.

`ROS Robots <https://robots.ros.org/>`_
---------------------------------------

:Purpose:
  A place to showcase robots that use ROS 1 or ROS 2.

:Notes:
  Robots on this page are community contributed, see the site for instructions on how to contribute a robot.

`ROS Wiki <http://wiki.ros.org/>`_
----------------------------------

:Purpose:
  ROS 1 documentation and user modifiable content.

:Notes:

  * We are not using the wiki for ROS 2 because of the overhead involved in moderating the wiki.
    The wiki's functionality for ROS 2 will be replaced by `ROS 2 Docs`, `ROS Discourse`_, and `ROS Answers`_
  * The wiki will active until at least the last ROS 1 distribution is EOL.

`ROS.org <https://www.ros.org/>`_
---------------------------------

:Purpose:
  The ROS 1 and ROS 2 product landing page, which gives a high-level description of ROS and links to other ROS sites.

Deprecated
----------

`ROS 2 Docs <https://docs.ros2.org>`_
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS 2 docs hosts API documentation for distributions up to and including Galactic.
From Humble and onwards, API documentation will be hosted at `ROS Docs`_.
