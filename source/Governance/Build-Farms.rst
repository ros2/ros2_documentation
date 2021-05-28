.. _BuildFarms:

===============
ROS Build Farms
===============

:Date: 2021-05-28
:Version: 0.1
:Organization: info@openrobotics.org

.. contents:: Table of Contents
   :depth: 2
   :local:

The ROS build farms are an important infrastructure to support the ROS ecosystem. It provides
building of source and binary packages, continuous integration, testing, and analysis.
There are two hosted instance for open source packages:

#. https://build.ros.org/ for ROS 1 packages
#. https://build.ros2.org/ for ROS 2 packages

If you are going to use any of the provided infrastructure please consider signing up for the build
farm discussion forum (http://discourse.ros.org/c/buildfarm) in order to receive notifications,
e.g., about any upcoming changes.

Jobs and Deployment
-------------------

The ROS build farms perform several different jobs. For each job type you will find a detailed
description what they do and how they work:

* `release jobs`_ generate binary packages, e.g., debian packages
* `devel jobs`_ build and test ROS packages within a single repository
* `CI jobs`_ build and test ROS packages across repositories with the option of using artifacts from other CI jobs to speed up the build
* `doc jobs`_ generate the API documentation of packages and extract information from the manifests
* `miscellaneous jobs`_ perform maintenance tasks and generate informational data to visualize the status of the build farm and its generated artifacts

**Creation/deployment** of the jobs happens when when packages are bloomed_, i.e. released for ROS
1 or ROS 2. Once blooming is successful and a package is incorporated in one of the ros
distributions (via pull request to rosdistro_), the according jobs will be spawned.

**Execution** of the jobs depends on the type of the job:

* `devel jobs`_ will be triggered every time a commit is done to the respective branch or pull request of the upstream [1]_ repository
* `release jobs`_ will be triggered once every time a new package version is released resp. a new rosdistro_ pull request was accepted for this package


Frequency Asked Questions (FaQ)
-------------------------------

Troubleshooting / Debugging
---------------------------

Further Reading
---------------

.. [1] That is the repository containing the source code of the respective ROS 1 / ROS 2 package.

.. _`release jobs`:
   https://github.com/ros-infrastructure/ros_buildfarm/blob/master/doc/jobs/release_jobs.rst
.. _`devel jobs`:
   https://github.com/ros-infrastructure/ros_buildfarm/blob/master/doc/jobs/devel_jobs.rst
.. _`CI jobs`:
   https://github.com/ros-infrastructure/ros_buildfarm/blob/master/doc/jobs/ci_jobs.rst
.. _`doc jobs`:
   https://github.com/ros-infrastructure/ros_buildfarm/blob/master/doc/jobs/doc_jobs.rst
.. _`miscellaneous jobs`:
   https://github.com/ros-infrastructure/ros_buildfarm/blob/master/doc/jobs/miscellaneous_jobs.rst
.. _bloomed:
   http://wiki.ros.org/bloom
.. _rosdistro:
   https://github.com/ros/rosdistro
