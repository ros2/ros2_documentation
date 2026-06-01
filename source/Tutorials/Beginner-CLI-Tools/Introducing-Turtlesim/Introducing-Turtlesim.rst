.. redirect-from::

    Tutorials/Turtlesim/Introducing-Turtlesim

.. _Turtlesim:

Setting up Turtlesim - how-to
=============================

The beginner ROS tutorials use Turtlesim, a simple simulator, to demonstrate core concepts without any real hardware.
This article walks you through installing Turtlesim along with the rqt GUI, then helps you run a short exercise to get hands-on experience with the basics.

**Area: ROS-tutorials | Content-type: how-to | Experience: beginner**

.. contents:: Contents
   :depth: 3
   :local:

Summary
-------

Turtlesim is a lightweight simulator for learning ROS basics.
It helps you practice core concepts such as nodes, topics, and services, before you can work with larger systems.

The rqt tool is a graphical interface for interacting with ROS.
You can do the same tasks from the command line, but rqt gives you a faster way to inspect and call services while learning.

Prerequisites
-------------

Before you start, make sure ROS is installed and your environment is configured.

To learn how to set up your environment, see :doc:`../Configuring-ROS2-Environment`.

Steps
-----

1 Install Turtlesim
^^^^^^^^^^^^^^^^^^^

Turtlesim is a lightweight simulator that represents a robot as a turtle you can control on the screen.
It lets you practice core ROS concepts without the need for any real hardware.

As always, start by sourcing your setup files in a new terminal, as described in the :doc:`previous tutorial <../Configuring-ROS2-Environment>`.

Install the Turtlesim package for your ROS distribution:

.. tabs::

  .. group-tab:: Ubuntu

      .. code-block:: console

        $ sudo apt update
        $ sudo apt install ros-{DISTRO}-turtlesim

  .. group-tab:: RHEL

      .. code-block:: console

        $ sudo dnf install ros-{DISTRO}-turtlesim

  .. group-tab:: macOS

      As long as the archive you installed ROS from contains the ``ros_tutorials`` repository, you should already have Turtlesim installed.

  .. group-tab:: Windows

      As long as the archive you installed ROS from contains the ``ros_tutorials`` repository, you should already have Turtlesim installed.

To check if the package is installed, run the following command, which should return a list of Turtlesim's executables:

.. code-block:: console

  $ ros2 pkg executables turtlesim
  turtlesim draw_square
  turtlesim mimic
  turtlesim turtle_teleop_key
  turtlesim turtlesim_node

2 Install rqt (recommended)
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The rqt tool is a graphical interface for ROS. Instead of typing commands by hand, you can use rqt to explore and call Turtlesim's services with a few clicks.

In a new terminal, install ``rqt`` and the common plugins:

.. tabs::

  .. group-tab:: Ubuntu

    .. code-block:: console

      $ sudo apt update
      $ sudo apt install ros-{DISTRO}-rqt ros-{DISTRO}-rqt-common-plugins

  .. group-tab:: RHEL

    .. code-block:: console

      $ sudo dnf install 'ros-{DISTRO}-rqt*'

  .. group-tab:: macOS

    The standard archive for installing ROS on macOS contains ``rqt`` and its plugins, so you should already have ``rqt`` installed.

  .. group-tab:: Windows

    The standard archive for installing ROS on Windows contains ``rqt`` and its plugins, so you should already have ``rqt`` installed.

3 Try out Turtlesim with rqt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following example uses rqt, but you can also operate Turtlesim by using the command line only.

#. Start Turtlesim:

   .. code-block:: console

      $ ros2 run turtlesim turtlesim_node
      [INFO] [turtlesim]: Starting turtlesim with node name /turtlesim
      [INFO] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]

   The simulator window appears with a random turtle in the center.

   .. image:: images/turtlesim.png

#. In a new terminal, source ROS again and start keyboard teleoperation:

   .. code-block:: console

      $ ros2 run turtlesim turtle_teleop_key

   Keep the ``turtle_teleop_key`` terminal active and use arrow keys to move the turtle.

   .. note::

      Pressing an arrow key will only cause the turtle to move a short distance and then stop.
      This is because, realistically, you wouldn't want a robot to continue carrying on an instruction if, for example, the operator lost the connection to the robot.

#. Optional: Inspect the running system:

   .. code-block:: console

      $ ros2 node list
      $ ros2 topic list
      $ ros2 service list
      $ ros2 action list

#. Start rqt:

   .. code-block:: console

      $ rqt

#. In rqt, select **Plugins** > **Services** > **Service Caller**.

   .. note::

      It may take some time for rqt to locate all the plugins.
      If you click on **Plugins** but don't see **Services** or any other options, close rqt and enter the command ``rqt --force-discover`` in your terminal.

   .. image:: images/rqt.png

#. Select the ``/spawn`` service, then enter:

   - ``name`` = ``turtle2``
   - ``x`` = ``1.0``
   - ``y`` = ``1.0``

   Click **Call** to spawn the second turtle.

   .. image:: images/spawn.png

   .. note::

      If you try to spawn a new turtle with the same name as an existing turtle, like the default ``turtle1``, you will get an error message in the terminal that runs ``turtlesim_node``:

      .. code-block:: console

         [ERROR] [turtlesim]: A turtle named [turtle1] already exists

#. Select the ``/turtle1/set_pen`` service, then set:

   - ``r`` = ``255``
   - ``width`` = ``5``

   Click **Call**, then use arrow keys in the teleop terminal to confirm the turtle draws a red, thicker line.

   .. image:: images/set_pen.png

   .. image:: images/new_pen.png

#. Control ``turtle2`` from another terminal by remapping:

   .. code-block:: console

      $ ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel --remap turtle1/rotate_absolute:=turtle2/rotate_absolute

   Keep this new terminal active to move ``turtle2``.
   Use the original teleop terminal to move ``turtle1``.

   .. image:: images/remap.png

#. Close Turtlesim:

   - In the ``turtlesim_node`` terminal, press ``Ctrl + C``.
   - In ``turtle_teleop_key`` terminals, press ``q``.

Next steps
----------

Now that you have Turtlesim and rqt running, you can continue with :doc:`../Understanding-ROS2-Nodes/Understanding-ROS2-Nodes`.

Related content
---------------

More articles:

* :doc:`../Configuring-ROS2-Environment`
* :doc:`../Understanding-ROS2-Nodes/Understanding-ROS2-Nodes`
* :doc:`../Understanding-ROS2-Topics/Understanding-ROS2-Topics`

Packages/reference:

* `Turtlesim package in ros_tutorials <https://github.com/ros/ros_tutorials/tree/{REPOS_FILE_BRANCH}/turtlesim>`__
* `rqt <https://index.ros.org/p/rqt/>`__
* `rqt_common_plugins <https://index.ros.org/p/rqt_common_plugins/>`__

FAQs
----

Do I need rqt to use Turtlesim?
   No. You can use Turtlesim entirely from the command line. The rqt tool is recommended because it helps you visualise the instructions you send to the turtle.

Why can I move only one turtle at a time?
   Each ``turtle_teleop_key`` instance controls one turtle namespace.
   To control ``turtle2``, run a second teleop node with topic and action remapping.

Why doesn't the turtle move when I press the arrow keys?
   The ``turtle_teleop_key`` terminal must be the active window for it to capture your key presses.
   Each press also moves the turtle only a short distance, so you need to press the keys repeatedly to keep the turtle moving.
