.. _ROS2Mirrors:

Using ROS 2 Mirrors
===================

.. contents:: Table of Contents
   :depth: 2
   :local:

This guide explains how to use ROS 2 mirrors to improve download speeds and ensure availability of ROS 2 packages.

Background
----------

ROS 2 mirrors are community-maintained copies of the official ROS 2 repositories.
They serve two primary purposes:

1. **Faster access**: Users geographically closer to a mirror may experience faster download speeds
2. **Redundancy**: Mirrors provide backup access when the primary servers are unavailable or experiencing high load

Available ROS 2 Mirrors
-----------------------

Package Repository Mirrors
^^^^^^^^^^^^^^^^^^^^^^^^^^

The Oregon State University Open Source Lab (OSUOSL) maintains official rsync endpoints for ROS 2 repositories:

.. list-table:: Official ROS 2 Repository Endpoints
   :header-rows: 1

   * - Repository
     - Description
     - Rsync Path
   * - ros2-main
     - Main ROS 2 Debian package repository
     - ``rsync.osuosl.org::ros2-main``
   * - ros2-testing
     - Testing ROS 2 Debian packages repository
     - ``rsync.osuosl.org::ros2-testing``
   * - ros2-rhel-main
     - Main ROS 2 RHEL packages repository
     - ``rsync.osuosl.org::ros2-rhel-main``
   * - ros2-rhel-testing
     - Testing ROS 2 RHEL packages repository
     - ``rsync.osuosl.org::ros2-rhel-testing``

Community Mirrors
^^^^^^^^^^^^^^^^^

The following community-maintained mirrors may provide faster access depending on your location:

**Asia**

* **Singapore**: ``mirror-ap.wiki.ros.org``
* **Tokyo, Japan**: ``opensource-robotics.tokyo.jp`` (TORK)
* **China**: ``roswiki.autolabor.com.cn`` (Autolabor)
* **China**: ``ros.exbot.net`` (ExBot Robotics Lab)
* **Vietnam**: ``mirrors.vinahost.vn/ros/`` (VinaHost)

**Europe**

* **Freiburg, Germany**: ``wiki.ros.org.ros.informatik.uni-freiburg.de``
* **Barcelona, Spain**: ``devel.iri.upc.edu`` (labrobotica)

**North America**

* **Maryland, USA**: ``mirror.umd.edu``
* **New York, USA**: ``mirror.clarkson.edu``

**South America**

* **Sao Paulo, Brazil**: ``ros.fei.edu.br`` (Centro Universitario da FEI)

.. note::

   Mirror availability and coverage may vary.
   Check with individual mirror maintainers for the most up-to-date information on supported distributions and packages.

Using a Mirror for Package Installation
---------------------------------------

To use an alternative mirror for ROS 2 package installation, you can modify your apt sources configuration.

1. Identify the closest or fastest mirror for your location from the list above.

2. Create or modify the ROS 2 sources list:

   .. code-block:: bash

      sudo nano /etc/apt/sources.list.d/ros2.list

3. Replace the default repository URL with your preferred mirror URL.

4. Update your package index:

   .. code-block:: bash

      sudo apt update

.. warning::

   When using community mirrors, ensure you trust the mirror maintainer.
   Official OSUOSL mirrors are recommended for security-sensitive environments.

Setting Up Your Own Mirror
--------------------------

If you want to host a ROS 2 mirror for your organization or community, you can use rsync to synchronize with the official repositories.

Prerequisites
^^^^^^^^^^^^^

* A server with sufficient storage (100+ GB recommended)
* rsync installed
* Adequate network bandwidth

Basic Rsync Command
^^^^^^^^^^^^^^^^^^^

To synchronize ROS 2 packages:

.. code-block:: bash

   rsync -avz --delete rsync.osuosl.org::ros2-main /path/to/local/mirror/ros2

Options explained:

* ``-a``: Archive mode (preserves permissions, timestamps, etc.)
* ``-v``: Verbose output
* ``-z``: Compress data during transfer
* ``--delete``: Remove files that no longer exist on the source

Automated Synchronization
^^^^^^^^^^^^^^^^^^^^^^^^^

For production mirrors, set up a cron job to synchronize regularly:

.. code-block:: bash

   # Add to crontab (crontab -e)
   # Sync every 6 hours
   0 */6 * * * rsync -avz --delete rsync.osuosl.org::ros2-main /var/www/ros2-mirror/main

Bandwidth Limiting
^^^^^^^^^^^^^^^^^^

To limit bandwidth usage during synchronization:

.. code-block:: bash

   rsync -avz --delete --bwlimit=500 rsync.osuosl.org::ros2-main /path/to/mirror

The ``--bwlimit`` option specifies the maximum transfer rate in KB/s.

Registering Your Mirror
-----------------------

If you maintain a public ROS 2 mirror and would like it listed in the official documentation, please:

1. Ensure your mirror is stable and regularly synchronized
2. Open an issue or pull request on the `ros2_documentation repository <https://github.com/ros2/ros2_documentation>`_
3. Include your mirror URL, geographic location, and contact information

Related Resources
-----------------

* `ROS Infrastructure Mirror Configuration <https://github.com/ros-infrastructure/mirror>`_
* `OSUOSL Mirror Information <https://osuosl.org/services/hosting/>`_
* :doc:`Installation Troubleshooting <Installation-Troubleshooting>`
