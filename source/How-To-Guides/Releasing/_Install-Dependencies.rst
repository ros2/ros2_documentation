Install tools that help the process of making the releases.
On debian-based systems such as Ubuntu, the recommended installation method is to use ``apt``.
On non-debian systems, use ``pip3``:

.. tabs::

   .. group-tab:: apt

      .. code-block:: bash

         sudo apt install python3-bloom python3-catkin-pkg

   .. group-tab:: pip3

      .. code-block:: bash

         pip3 install -U bloom catkin_pkg
