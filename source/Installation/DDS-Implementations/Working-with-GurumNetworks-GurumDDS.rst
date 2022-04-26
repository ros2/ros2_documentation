.. redirect-from::

    Working-with-GurumNetworks-GurumDDS

GurumNetworks GurumDDS
======================

rmw_gurumdds is a implementation of the ROS middleware interface using GurumNetworks GurumDDS.
More information about GurumDDS is available on our website: https://gurum.cc/index_eng


Prerequisites
-------------

The following description assumes that you have completed the 'Environment setup' process
from the :doc:`Installing ROS 2 via Debian Packages <../Ubuntu-Install-Debians>` or
from the :doc:`Building ROS 2 on Ubuntu Linux <../Ubuntu-Development-Setup>`.

rmw_gurumdds requires version of GurumDDS-2.7.x.
Debian packages of GurumDDS is provided in the ROS 2 apt repositories on linux.
Windows binary installer of GurumDDS will be supported soon.

GurumDDS requires a license. See the next page: https://gurum.cc/free_trial_eng.html
After getting a license, move it to the following location.

=============  ================
 DDS Version   License Location
=============  ================
<= 2.7.2860    /etc/flame
>= 2.7.2861    /etc/gurumnet
=============  ================


Install packages
----------------

The easiest way is to install from ROS 2 apt repository.
When ros-{DISTRO}-rmw-gurumdds-cpp is installed, gurumdds-2.7 is also installed.

.. code-block:: bash

   sudo apt install ros-{DISTRO}-rmw-gurumdds-cpp

Build from source code
----------------------

Building from source code is also another way to install.

First, clone rosidl_typesupport_gurumdds and rmw_gurumdds in the ROS 2 workspace source directory.

.. code-block:: bash

   cd ros2_ws/src
   git clone https://github.com/ros2/rmw_gurumdds ros2/rmw_gurumdds
   git clone https://github.com/ros2/rosidl_typesupport_gurumdds ros2/rosidl_typesupport_gurumdds

Then, install necessary packages for GurumDDS.

.. code-block:: bash

   cd ..
   rosdep install --from src -i --rosdistro {DISTRO}

Finally, run colcon build.

.. code-block:: bash

   colcon build --symlink-install

Switch to rmw_gurumdds
------------------------

Switch from other rmw to rmw_gurumdds by specifying the environment variable.

.. code-block:: bash

   export RMW_IMPLEMENTATION=rmw_gurumdds_cpp

See also: :doc:`Working with multiple RMW implementations <../../How-To-Guides/Working-with-multiple-RMW-implementations>`

Run the talker and listener
---------------------------

Now run ``talker`` and ``listener`` to test GurumDDS.
Don't forget to set up environment by setup script.

.. code-block:: bash

   ros2 run demo_nodes_cpp talker

.. code-block:: bash

   ros2 run demo_nodes_cpp listener
