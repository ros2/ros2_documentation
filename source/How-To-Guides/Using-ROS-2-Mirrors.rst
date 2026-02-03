Using ROS 2 Mirrors
===================

ROS 2 packages and documentation are hosted on official servers, but community-maintained mirrors are available to improve download speeds for users around the world.
Using a mirror closer to your geographic location can significantly reduce download times when installing ROS 2 packages.

This guide provides information on available mirrors and how to configure your system to use them.

Why use a mirror?
-----------------

Mirrors offer several benefits:

* **Faster downloads**: Mirrors located closer to you geographically typically provide faster download speeds.
* **Reduced latency**: Local mirrors reduce network latency compared to distant official servers.
* **Load distribution**: Mirrors help distribute the load on the official ROS 2 servers.
* **Reliability**: If the official server is experiencing issues, mirrors can provide continued access.

APT repository mirrors (Ubuntu)
-------------------------------

For Ubuntu users installing ROS 2 via APT packages, you can configure your system to use a mirror of the official ROS 2 APT repository.

Available APT mirrors
^^^^^^^^^^^^^^^^^^^^^

The following community-maintained mirrors are available for the ROS 2 APT repository:

.. list-table::
   :header-rows: 1
   :widths: 20 30 30 20

   * - Region
     - Mirror URL
     - Maintainer
     - Notes
   * - Asia (Singapore)
     - ``http://mirror-ap.wiki.ros.org/``
     - Tully Foote
     - OSRF maintained
   * - Japan (Tokyo)
     - ``http://opensource-robotics.tokyo.jp/ros.org/``
     - TORK
     - Japan region
   * - China
     - ``http://roswiki.autolabor.com.cn``
     - Autolabor
     - Mainland China
   * - China
     - ``http://ros.exbot.net/``
     - ExBot Robotics Lab
     - Alternative China mirror
   * - Vietnam
     - ``http://mirrors.vinahost.vn/ros/``
     - VinaHost
     - Vietnam region
   * - Germany (Freiburg)
     - ``http://wiki.ros.org.ros.informatik.uni-freiburg.de/``
     - University of Freiburg
     - Germany region
   * - Spain (Barcelona)
     - ``https://devel.iri.upc.edu/docs/roswiki/``
     - IRI Lab, UPC
     - Spain/Catalonia region
   * - USA (Maryland)
     - ``http://mirror.umd.edu/roswiki/``
     - University of Maryland
     - East Coast USA
   * - USA (New York)
     - ``http://mirror.clarkson.edu/ros/``
     - Clarkson University
     - Northeast USA
   * - USA (California)
     - ``https://ghostarchive.org/ros/``
     - Ghostarchive
     - West Coast USA
   * - Brazil (Sao Paulo)
     - ``http://ros.fei.edu.br``
     - Centro Universitario da FEI
     - South America

Configuring an APT mirror
^^^^^^^^^^^^^^^^^^^^^^^^^

To use a mirror for APT package downloads, you can modify your APT sources list.

1. Open your ROS 2 sources list file:

   .. code-block:: bash

      sudo nano /etc/apt/sources.list.d/ros2.list

2. Replace the default URL with your preferred mirror URL.

3. Update your package lists:

   .. code-block:: bash

      sudo apt update

.. warning::

   Mirrors may not always be in sync with the official repository.
   If you encounter package version issues, try switching back to the official repository.

Testing mirror speed
--------------------

Before selecting a mirror, you can test the download speed to find the fastest option for your location.
You can use tools like ``curl`` or ``wget`` to test download speeds:

.. code-block:: bash

   # Test download speed from a mirror
   curl -o /dev/null -w "Speed: %{speed_download} bytes/sec\n" <mirror-url>/test-file

Hosting a mirror
----------------

If you are interested in hosting a ROS 2 mirror to benefit your local community, please see the community discussions on `ROS Discourse <https://discourse.openrobotics.org/c/mirrors>`__.

Requirements for hosting a mirror typically include:

* Sufficient bandwidth and storage capacity
* Regular synchronization with the official repository
* Commitment to long-term maintenance

Related resources
-----------------

* `ROS Discourse Mirror Discussion <https://discourse.openrobotics.org/c/mirrors>`__
* :doc:`Installation Troubleshooting </How-To-Guides/Installation-Troubleshooting>`
* :doc:`Ubuntu Installation Guide </Installation/Ubuntu-Install-Debs>`
