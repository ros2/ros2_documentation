.. redirect-from::

    Installation/ROS-2-Mirrors


Mirrors
=======

.. contents:: Table of Contents
   :depth: 3

Docs Mirrors
------------

Mirrors of the ROS Docs act as a backup when the `main site <http://docs.ros.org>`_ is unavailable, and may provide faster access to users who are geographically closer to the mirror.

Debian/Ubuntu (APT) Repository Mirrors
--------------------------------------

To use these mirrors, replace the official ROS repository URL with the one listed below in your APT configuration.

Asia
^^^^

.. list-table::
   :widths: 30 20 50

   * - Tsinghua University (TUNA)
     - China
     - `https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/ <https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/>`_
   * - USTC
     - China
     - `https://mirrors.ustc.edu.cn/ros2/ubuntu/ <https://mirrors.ustc.edu.cn/ros2/ubuntu/>`_
   * - Alibaba Cloud (Aliyun)
     - China
     - `https://mirrors.aliyun.com/ros2/ubuntu/ <https://mirrors.aliyun.com/ros2/ubuntu/>`_
   * - Qilu University of Technology (QLU)
     - China
     - `https://mirrors.qlu.edu.cn/ros2/ubuntu/ <https://mirrors.qlu.edu.cn/ros2/ubuntu/>`_
   * - Chongqing University (CQU)
     - China
     - `https://mirrors.cqu.edu.cn/ros2/ubuntu/ <https://mirrors.cqu.edu.cn/ros2/ubuntu/>`_

Europe
^^^^^^

.. list-table::
   :widths: 30 20 50

   * - Delft University of Technology
     - the Netherlands
     - `http://ftp.tudelft.nl/ros2/ubuntu/ <http://ftp.tudelft.nl/ros2/ubuntu/>`_

North America
^^^^^^^^^^^^^

.. list-table::
   :widths: 30 20 50

   * - University of Maryland (UMD)
     - USA
     - `http://mirror.umd.edu/packages.ros.org/ros2/ubuntu/ <http://mirror.umd.edu/packages.ros.org/ros2/ubuntu/>`_
   * - nulled LLC
     - USA
     - `http://mirror.nulled.llc/ros2/ubuntu/ <http://mirror.nulled.llc/ros2/ubuntu/>`_

Oceania
^^^^^^^

.. list-table::
   :widths: 30 20 50

   * - AARNet
     - Australia
     - `https://mirror.aarnet.edu.au/pub/ros2-packages/ubuntu/ <https://mirror.aarnet.edu.au/pub/ros2-packages/ubuntu/>`_

South America and Africa
^^^^^^^^^^^^^^^^^^^^^^^^

There are currently no officially verified ROS 2 mirrors for these regions.
If you are hosting a mirror in South America or Africa and would like it listed here, please see the **Hosting a Mirror** section below.

Creating a mirror
-----------------

If you are maintaining a mirror please join the Mirrors category on discourse.openrobotics.org: `https://discourse.openrobotics.org/c/infrastructure-project/infra-mirrors/ <https://discourse.openrobotics.org/c/infrastructure-project/infra-mirrors/>`_ for both feedback and prompt updates.

Using a Mirror
^^^^^^^^^^^^^^

To use a mirror, replace ``packages.ros.org`` with the mirror URL in your ``ros2-latest.list`` file:

.. code-block:: bash

   # Example for TUNA mirror
   sudo sed -i 's|http://packages.ros.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' /etc/apt/sources.list.d/ros2-latest.list
   sudo apt update

Setting up a Mirror
-------------------

The ROS infrastructure uses ``rsync`` to distribute packages.
To create a local mirror of the ROS 2 repositories:

1. **Storage Requirement:** Ensure you have at least 500GB of available disk space.
2. **Sync Command:** Use ``rsync`` to pull from the official OSUOSL endpoints:

.. code-block:: bash

   # Sync the main ROS 2 repository
   rsync -azv rsync.osuosl.org::ros2-main /your/local/path --delete

3. **Maintenance:** Set up a ``cron`` job to sync every 6-12 hours.

Adding your mirror to this list
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To be officially listed, your mirror must meet the following requirements:

* Support **HTTPS**.
* Sync at least once every 24 hours.
* Provide a contact email for infrastructure alerts.

Once verified, please open a Pull Request against this page or post in the `Mirrors Discourse <https://discourse.openrobotics.org/c/infrastructure-project/infra-mirrors/>`_.

Mirroring docs.ros.org
----------------------

Mirroring the documentation site requires specific configuration to prevent search engine fragmentation.
If you are interested in hosting a regional mirror of the documentation, please **contact the infrastructure team** via Discourse before proceeding.
