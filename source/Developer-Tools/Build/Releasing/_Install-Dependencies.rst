.. redirect-from::

    How-To-Guides/Releasing/_Install-Dependencies

.. meta::
   :contentType: how-to
   :experience:
   :area: builds, tools
   :distribution: {DISTRO}
   :product: {PRODUCT}

Installing dependencies - how-to
================================

.. short-description::
   Release workflows require Bloom, `catkin_pkg`, and an initialised `rosdep` setup before you continue.
   This article describes how to install the required release tools on deb, RPM, and other platforms.
   After you follow these steps, your environment will be ready for the next release task.

.. showmeta::
    :order: area, contentType, experience
    :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Table of Contents
   :depth: 2
   :local:

Summary
-------

Install the release tools for your platform.

For deb platforms such as Ubuntu, use:

.. code-block:: console

   $ sudo apt install python3-bloom python3-catkin-pkg

For RPM platforms such as RHEL, use:

.. code-block:: console

   $ sudo dnf install python3-bloom python3-catkin_pkg

For other platforms, use:

.. code-block:: console

   $ pip3 install -U bloom catkin_pkg

Initialise and update `rosdep` with sudo `rosdep init` and `rosdep update`.
If `rosdep init` has already been run, the error can safely be ignored.

Install tools that you will use in the upcoming steps according to your platform:

.. tabs::

   .. group-tab:: deb (e.g. Ubuntu)

      .. code-block:: console

         $ sudo apt install python3-bloom python3-catkin-pkg

   .. group-tab:: RPM (e.g. RHEL)

      .. code-block:: console

          $ sudo dnf install python3-bloom python3-catkin_pkg

   .. group-tab:: Other

      .. code-block:: console

         $ pip3 install -U bloom catkin_pkg

Make sure you have rosdep initialized:

.. code-block:: console

    $ sudo rosdep init
    $ rosdep update

Note that the ``rosdep init`` command may fail if it has already been initialized in the past; this can safely be ignored.
