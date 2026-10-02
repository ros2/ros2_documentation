.. redirect-from::

    Tutorials/Turtlesim/Introducing-Turtlesim

.. _Turtlesim:

Setting up Turtlesim - how-to
=============================

Turtlesim is a simple simulator to demonstrate core ROS concepts without any hardware.
This article walks you through installation of Turtlesim and the rqt GUI.
A short exercise provides you with basic experience with these tools.

**Area: Framework | Content-type: how-to | Experience: beginner**

.. contents:: Contents
   :depth: 3
   :local:

Summary
-------

Turtlesim is a lightweight simulator for learning ROS basics.
Install the package ``ros-{DISTRO}-turtlesim``.

The rqt tool is a graphical interface for interacting with ROS.
Install the package ``ros-{DISTRO}-rqt``.

Prerequisites
-------------

Before you start, make sure ROS is installed and your environment is configured.

To learn how to set up your environment, see :doc:`../Configuring-ROS2-Environment`.

Steps
-----

.. note::
    Do not forget to source ROS in every new terminal you open.
    See :doc:`Configuring environment <../Configuring-ROS2-Environment>`.

1 Install Turtlesim
^^^^^^^^^^^^^^^^^^^

Turtlesim is a lightweight simulator that represents a robot as a turtle you can control on the screen.
It lets you practice core ROS concepts without the need for any real hardware.

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

To check if the package is installed, run:

.. code-block:: console

  $ ros2 pkg executables turtlesim

The output should list Turtlesim's executables:

.. code-block:: console

  turtlesim draw_square
  turtlesim mimic
  turtlesim turtle_teleop_key
  turtlesim turtlesim_node

2 Install rqt (recommended)
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The rqt tool is a graphical interface for ROS.
Instead of typing commands by hand, you can use rqt to explore and call Turtlesim's services with a few clicks.
For more information about rqt, see :doc:`../../Developer-Tools/Visualization/About-RQt`.

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

To check if the packages are installed, in a terminal, run:

.. code-block:: console

  $ ros2 pkg list | grep -E '^rqt_gui$|^rqt_common_plugins$'

The output should include:

.. code-block:: console

  rqt_common_plugins
  rqt_gui

.. note::

   If the packages do not appear in the list, make sure you have sourced ROS in the current terminal.

3 Try out Turtlesim with rqt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following example uses rqt but you can also operate Turtlesim by entering the same commands in the command line.

#. Start Turtlesim:

   .. code-block:: console

      $ ros2 run turtlesim turtlesim_node
      [INFO] [turtlesim]: Starting turtlesim with node name /turtlesim
      [INFO] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]

   The simulator window opens with a random turtle in the center.

   .. image:: images/turtlesim.png

#. In a new terminal, source ROS again and start keyboard teleoperation:

   .. code-block:: console

      $ ros2 run turtlesim turtle_teleop_key

   Keep the ``turtle_teleop_key`` terminal active and use arrow keys to move the turtle.

   .. note::

      Pressing an arrow key causes the turtle to move a short distance and then stop.
      This behaviour ensures a robot doesn't continue carrying on an instruction if, for example, the operator loses connection to the robot.

#. (Optional) List the active nodes, topics, services, and actions.
   This will help you verify what is currently running and available.

   In a new terminal, source ROS again, then run:

   .. code-block:: console

      $ ros2 node list
      $ ros2 topic list
      $ ros2 service list
      $ ros2 action list

   ``ros2 node list`` shows the nodes currently running in the system.
   ``ros2 topic list`` shows the topics through which nodes exchange data.
   ``ros2 service list`` shows the services nodes expose for request-response interactions.
   ``ros2 action list`` shows the actions available for long-running tasks such as rotating the turtle.

#. Start rqt:

   .. code-block:: console

      $ rqt

#. In rqt, select **Plugins** > **Services** > **Service Caller**.

   .. note::

      It may take some time for rqt to locate all the plugins.
      If you click on **Plugins** but don't see **Services** or any other options, close rqt and enter the command ``rqt --force-discover`` in your terminal.

   .. note::

      You might need to select the **Refresh Service** button to the left of the **Service** drop-down.

   .. image:: images/rqt.png

#. To spawn a second turtle, select the ``/spawn`` service, then enter:

   - ``name`` = ``turtle2``
   - ``x`` = ``1.0``
   - ``y`` = ``1.0``

   Click **Call**.

   .. image:: images/spawn.png

   .. note::

      Each turtle needs a unique name.
      If you spawn a turtle with an existing name, you get an error message in the terminal that runs ``turtlesim_node``:

      .. code-block:: console

         [ERROR] [turtlesim]: A turtle named [turtle1] already exists

#. Set the turtle to draw a red, thicker line.
   Select the ``/turtle1/set_pen`` service, then set:

   - ``r`` = ``255``
   - ``width`` = ``5``

   Click **Call**, then use arrow keys in the teleoperation terminal to confirm the turtle draws a red, thicker line.

   .. image:: images/set_pen.png

   .. image:: images/new_pen.png

#. Set up a teleoperation node for ``turtle2``.

   By default, ``turtle_teleop_key`` sends commands to topics and actions under the ``turtle1`` namespace.
   To control ``turtle2`` instead, use the ``--remap`` option to point those topics and actions at the ``turtle2`` namespace.
   This lets you reuse the same executable for a different turtle without changing any code.

   In a new terminal, run:

   .. code-block:: console

      $ ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel --remap turtle1/rotate_absolute:=turtle2/rotate_absolute

   Keep this new terminal active to move ``turtle2``.
   Continue using the original teleoperation terminal to move ``turtle1``.

   .. image:: images/remap.png

#. Close Turtlesim:

   - In the ``turtlesim_node`` terminal, press ``Ctrl + C``.
   - In ``turtle_teleop_key`` terminals, press ``q``.

Related content
---------------

More articles:

* :doc:`First steps with ROS <../../First-Steps>`
* :doc:`../Configuring-ROS2-Environment`
* :doc:`../../ROS-Framework/nodes/Working-with-nodes/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes`
* :doc:`../../ROS-Framework/interfaces/topics/Understanding-ROS2-Topics/Understanding-ROS2-Topics`

Packages/reference:

* `Turtlesim package in ros_tutorials <https://github.com/ros/ros_tutorials/tree/{REPOS_FILE_BRANCH}/turtlesim>`__
* `rqt <https://index.ros.org/p/rqt/>`__
* `rqt_common_plugins <https://index.ros.org/p/rqt_common_plugins/>`__

FAQs
----

Do I need rqt to use Turtlesim?
   No.
   You can use Turtlesim entirely from the command line.
   The rqt tool is recommended because it helps you visualize the instructions you send to the turtle.

Why can I move only one turtle at a time?
   Each ``turtle_teleop_key`` instance controls one turtle namespace.
   To control ``turtle2``, run a second teleop node with topic and action remapping.

Why doesn't the turtle move when I press the arrow keys?
   The ``turtle_teleop_key`` terminal must be the active window for it to capture your key presses.
   Each press also moves the turtle only a short distance, so you need to press the keys repeatedly to keep the turtle moving.
