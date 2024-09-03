Install tools that you will use in the upcoming steps according to your platform:

.. tabs::

   .. group-tab:: deb (eg. Ubuntu)

      .. code-block:: bash

         sudo apt install python3-bloom python3-catkin-pkg

   .. group-tab:: RPM (eg. RHEL)

      .. code-block:: bash

          sudo dnf install python3-bloom python3-catkin_pkg

   .. group-tab:: Other

      .. code-block:: bash

         pip3 install -U bloom catkin_pkg

Make sure you have rosdep initialized:

.. code-block:: bash

    sudo rosdep init
    rosdep update

Note that the ``rosdep init`` command may fail if it has already been initialized in the past; this can safely be ignored.
