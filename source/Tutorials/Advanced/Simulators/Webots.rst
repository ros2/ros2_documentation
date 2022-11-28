.. redirect-from::

    Tutorials/Simulators/Webots/Setting-up-a-Robot-Simulation-Webots

.. _Simulators:

Setting up a robot simulation (Webots)
======================================

**Goal:** Setup a robot simulation and control it from ROS 2.

**Tutorial level:** Advanced

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

Several robot simulators can be used with ROS 2, such as Gazebo, Webots, etc.
Unlike turtlesim, they provide fairly realistic results relying on physics-based models for robots, sensors, actuators and objects.
Hence, what you observe in simulation is very close to what you will get when transferring your ROS 2 controllers to a real robot.
In this tutorial, you are going to use the Webots robot simulator to set-up and run a very simple ROS 2 simulation scenario.

The ``webots_ros2`` package provides an interface between ROS 2 and Webots.
It includes several sub-packages, but in this tutorial, you are going to use only the ``webots_ros2_driver`` sub-package to implement a Python plugin controlling a simulated robot.
Some other sub-packages contain demos with different robots such as the TurtleBot3.
They are documented in the :doc:`../../Demos` page.

Prerequisites
-------------

It is recommended to understand basic ROS principles covered in the beginner :doc:`../../../Tutorials`.
In particular, :doc:`../../Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim`, :doc:`../../Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics`, :doc:`../../Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace`, :doc:`../../Beginner-Client-Libraries/Creating-Your-First-ROS2-Package` and :doc:`../../Intermediate/Launch/Creating-Launch-Files` are useful prerequisites.

.. tabs::

    .. group-tab:: Linux

        The Linux and ROS commands of this tutorial can be run in a standard Linux terminal.
        See the `Webots ROS 2 Linux installation instructions <https://github.com/cyberbotics/webots_ros2/wiki/Linux-Installation-Guide>`_.

    .. group-tab:: Windows

        The Linux and ROS commands of this tutorial must be run in a WSL (Windows Subsystem for Linux) environment.
        See the `Webots ROS 2 Windows installation instructions <https://github.com/cyberbotics/webots_ros2/wiki/Windows-Installation-Guide>`_.

    .. group-tab:: macOS

        The Linux and ROS commands of this tutorial must be run in a custom Docker container configured with the ``webots_ros2_driver`` package.
        See the `Webots ROS 2 macOS installation instructions <https://github.com/cyberbotics/webots_ros2/wiki/macOS-Installation-Guide>`_.

To install ``webots_ros2_driver`` from a terminal, proceed with the following commands.

.. code-block:: console

        sudo apt update
        sudo apt install ros-{DISTRO}-webots-ros2-driver
        source /opt/ros/{DISTRO}/setup.bash

.. note::

    If you want to install the whole ``webots_ros2`` package, follow these `instructions <https://github.com/cyberbotics/webots_ros2/wiki/Getting-Started>`_.

Tasks
-----

1 Create the package structure
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Let's organize the code in a custom ROS 2 package.
Create a new package named ``my_package`` from the ``src`` folder of your ROS 2 workspace.
Change the current directory of your terminal to ``ros2_ws/src`` and run:

.. code-block:: console

        ros2 pkg create --build-type ament_python --node-name my_robot_driver my_package --dependencies rclpy geometry_msgs webots_ros2_driver

The ``--node-name my_robot_driver`` option will create a ``my_robot_driver.py`` template Python plugin in the ``my_package`` subfolder that you will modify later.
The ``--dependencies rclpy geometry_msgs webots_ros2_driver`` option specifies the packages needed by the ``my_robot_driver.py`` plugin in the ``package.xml`` file.
Let's add a ``launch`` and a ``worlds`` folder inside the ``my_package`` folder.

.. code-block:: console

        cd my_package
        mkdir launch
        mkdir worlds

You should end up with the following folder structure:

.. code-block:: console

          src/
          └── my_package/
              ├── launch/
              ├── my_package/
              │   ├── __init__.py
              │   └── my_robot_driver.py
              ├── resource/
              │   └── my_package
              ├── test/
              │   ├── test_copyright.py
              │   ├── test_flake8.py
              │   └── test_pep257.py
              ├── worlds/
              ├── package.xml
              ├── setup.cfg
              └── setup.py

2 Setup the simulation world
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You will need a world file containing a robot to launch your simulation.
:download:`Download this world file <Code/my_world.wbt>` and move it inside ``my_package/worlds/``.

This is actually a fairly simple text file you can visualize in a text editor.
A simple robot is already included in this ``my_world.wbt`` world file.

.. note::

    In case you want to learn how to create your own robot model in Webots, you can check this `tutorial <https://github.com/cyberbotics/webots_ros2/wiki/Tutorial-Create-Webots-Robot>`_.

3 Change the my_robot_driver.py file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``webots_ros2_driver`` sub-package automatically creates a ROS 2 interface for most sensors.
In this task, you will extend this interface by changing the ``my_robot_driver.py`` file.

.. note::

    The purpose of this tutorial is to show a basic example with a minimum number of dependencies.
    However, you could avoid the use of this Python plugin by using another ``webots_ros2`` sub-package named ``webots_ros2_control``, introducing a new dependency.
    This other sub-package creates an interface with the ``ros2_control`` package that facilitates the control of a differential wheeled robot.

Open ``my_package/my_package/my_robot_driver.py`` in your favorite editor and replace its contents with the following:

.. literalinclude:: Code/my_robot_driver.py
    :language: python

As you can see, the ``MyRobotDriver`` class implements three methods.

The first method, named ``init(self, ...)``, is actually the ROS node counterpart of the Python ``__init__(self, ...)`` constructor.
It first gets the robot instance from the simulation (which can be used to access the `Webots robot API <https://cyberbotics.com/doc/reference/robot?tab-language=python>`_).
Then, it gets the two motor instances and initializes them with a target position and a target velocity.
Finally a ROS node is created and a callback method is registered for a ROS topic named ``/cmd_vel`` that will handle ``Twist`` messages.

.. literalinclude:: Code/my_robot_driver.py
    :language: python
    :dedent: 4
    :lines: 8-24

Then comes the implementation of the ``__cmd_vel_callback(self, twist)`` callback private method that will be called for each ``Twist`` message received on the ``/cmd_vel`` topic and will save it in the ``self.__target_twist`` member variable.

.. literalinclude:: Code/my_robot_driver.py
    :language: python
    :dedent: 4
    :lines: 26-27

Finally, the ``step(self)`` method is called at every time step of the simulation.
The call to ``rclpy.spin_once()`` is needed to keep the ROS node running smoothly.
At each time step, the method will retrieve the desired ``forward_speed`` and ``angular_speed`` from ``self.__target_twist``.
As the motors are controlled with angular velocities, the method will then convert the ``forward_speed`` and ``angular_speed`` into individual commands for each wheel.
This conversion depends on the structure of the robot, more specifically on the radius of the wheel and the distance between them.

.. literalinclude:: Code/my_robot_driver.py
    :language: python
    :dedent: 4
    :lines: 29-39

4 Create the my_robot.urdf file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You now have to create a URDF file to declare the ``my_robot_driver.py`` Python plugin.
This will allow the ``webots_ros2_driver`` ROS node to launch the plugin.

In the ``my_package/resource`` folder create a text file named ``my_robot.urdf`` with this contents:

.. literalinclude:: Code/my_robot.urdf
    :language: xml

.. note::

    This simple URDF file doesn't contain any link or joint information about the robot as it is not needed in this tutorial.
    However, URDF files usually contain much more information as explained in the :doc:`../../Intermediate/URDF/URDF-Main`.

5 Create the launch file
^^^^^^^^^^^^^^^^^^^^^^^^

Let's create now the launch file to easily launch the simulation and the ROS controller with a single command.
In the ``my_package/launch`` folder create a new text file named ``robot_launch.py`` with this code:

.. literalinclude:: Code/robot_launch.py
    :language: python

The ``WebotsLauncher`` object is a custom action that allows you to start a Webots simulation instance.
You have to specify in the constructor which world file the simulator will open.

.. literalinclude:: Code/robot_launch.py
    :language: python
    :dedent: 4
    :lines: 15-17

A supervisor Robot is always automatically added to the world file by ``WebotsLauncher``.
This robot is controlled by the custom node ``Ros2Supervisor``, which must also be started using the ``Ros2SupervisorLauncher``.
This node allows to spawn URDF robots directly into the world, and it also publishes useful topics like ``/clock``.

.. literalinclude:: Code/robot_launch.py
    :language: python
    :dedent: 4
    :lines: 19

Then, the ROS node interacting with the simulated robot is created.
This node, named ``driver``, is located in the ``webots_ros2_driver`` package.

.. tabs::

    .. group-tab:: Linux

        The node will be able to communicate with the simulated robot by using a custom protocol based on IPC and shared memory.

    .. group-tab:: Windows

        The node (in WSL) will be able to communicate with the simulated robot (in Webots on Windows) through a TCP connection.

    .. group-tab:: macOS

        The node (in the docker container) will be able to communicate with the simulated robot (in Webots on macOS) through a TCP connection.


In your case, you need to run a single instance of this node, because you have a single robot in the simulation.
But if you had more robots in the simulation, you would have to run one instance of this node per robot.
``WEBOTS_CONTROLLER_URL`` is used to define the name of the robot the driver should connect to.
The ``controller_url_prefix()`` method is mandatory, as it allows ``webots_ros2_driver`` to add the correct protocol prefix depending on your platform.
The ``robot_description`` parameter holds the contents of the URDF file which refers to the ``my_robot_driver.py`` Python plugin.

.. literalinclude:: Code/robot_launch.py
    :language: python
    :dedent: 4
    :lines: 21-29

After that, the three nodes are set to be launched in the ``LaunchDescription`` constructor:

.. literalinclude:: Code/robot_launch.py
    :language: python
    :dedent: 4
    :lines: 31-34

Finally, an optional part is added in order to shutdown all the nodes once Webots terminates (e.g., when it gets closed from the graphical user interface).

.. literalinclude:: Code/robot_launch.py
    :language: python
    :dedent: 8
    :lines: 35-40

6 Modify the setup.py file
^^^^^^^^^^^^^^^^^^^^^^^^^^

Finally, before you can start the launch file, you have to modify the ``setup.py`` file to include the extra files you added.
Open ``my_package/setup.py`` and replace its contents with:

.. literalinclude:: Code/setup.py
    :language: python

This sets-up the package and adds in the ``data_files`` variable the newly added files: ``my_world.wbt``, ``my_robot.urdf`` and ``robot_launch.py``.

7 Test the code
^^^^^^^^^^^^^^^


.. tabs::

    .. group-tab:: Linux

        From a terminal in your ROS 2 workspace run:

        .. code-block:: console

            colcon build
            source install/local_setup.bash
            ros2 launch my_package robot_launch.py

        This will launch the simulation.
        Webots will be automatically installed on the first run in case it was not already installed.

    .. group-tab:: Windows

        From a terminal in your WSL ROS 2 workspace run:

        .. code-block:: console

            colcon build
            export WEBOTS_HOME=/mnt/c/Program\ Files/Webots
            source install/local_setup.bash
            ros2 launch my_package robot_launch.py

        Be sure to use the ``/mnt`` prefix in front of your path to the Webots installation folder to access the Windows file system from WSL.

        This will launch the simulation.
        Webots will be automatically installed on the first run in case it was not already installed.

    .. group-tab:: macOS

        On macOS, a local server must be started on the host to start Webots from the Docker container.
        The local server can be downloaded `on the webots-server repository <https://github.com/cyberbotics/webots-server/blob/main/local_simulation_server.py>`_.

        In a terminal of the host machine (not in the container), specify the Webots installation folder (e.g. ``/Applications/Webots.app``) and start the server using the following commands:

        .. code-block:: console

            export WEBOTS_HOME=/Applications/Webots.app
            python3 local_simulation_server.py

        From the terminal of the Docker container, build and launch your custom package with:

        .. code-block:: console

            cd ~/ros2_ws
            colcon build
            source install/local_setup.bash
            ros2 launch my_package robot_launch.py


.. note::

    If you want to install Webots manually, you can download it `here <https://github.com/cyberbotics/webots/releases/latest>`_.


Then, open a second terminal and send a command with:

.. code-block:: console

            ros2 topic pub /cmd_vel geometry_msgs/Twist  "linear: { x: 0.1 }"

The robot is now moving forward.

.. image:: Image/Robot_moving_forward.png

At this point, the robot is able to blindly follow your motor commands.
But it will eventually bump into the wall as you order it to move forwards.

.. image:: Image/Robot_colliding_wall.png

To prevent this, let's use the sensors of the robot to detect the obstacles and avoid them.
Close the Webots window, this should also shutdown your ROS nodes started from the launcher.
Close also the topic command with ``Ctrl+C`` in the second terminal.

8 Updating my_robot.urdf
^^^^^^^^^^^^^^^^^^^^^^^^

You have to modify the URDF file in order to enable the sensors.
In ``my_robot.urdf`` replace the whole contents with:

.. literalinclude:: Code/my_robot_with_sensors.urdf
    :language: xml

The ROS 2 interface will parse the ``<device>`` tags referring to the **DistanceSensor** nodes and use the standard parameters in the ``<ros>`` tags to enable the sensors and name their topics.

9 Creating a ROS node to avoid obstacles
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The robot will use a standard ROS node to detect the wall and send motor commands to avoid it.
In the ``my_package/my_package/`` folder, create a file named ``obstacle_avoider.py`` with this code:

.. literalinclude:: Code/obstacle_avoider.py
    :language: python

This node will create a publisher for the command and subscribe to the sensors topics here:

.. literalinclude:: Code/obstacle_avoider.py
    :language: python
    :dedent: 8
    :lines: 14-17

When a measurement is received from the left sensor it will be copied to a member field:

.. literalinclude:: Code/obstacle_avoider.py
    :language: python
    :dedent: 4
    :lines: 19-20

Finally, a message will be sent to the ``/cmd_vel`` topic when a measurement from the right sensor is received.
The ``command_message`` will register at least a forward speed in ``linear.x`` in order to make the robot move when no obstacle is detected.
If any of the two sensors detect an obstacle, ``command_message`` will also register a rotational speed in ``angular.z`` in order to make the robot turn right.

.. literalinclude:: Code/obstacle_avoider.py
    :language: python
    :dedent: 4
    :lines: 22-32

10 Updating setup.py and robot_launch.py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You have to modify these two other files to launch your new node.
Edit ``setup.py`` and replace ``'console_scripts'`` with:

.. literalinclude:: Code/setup_sensor.py
    :language: python
    :dedent: 8
    :lines: 24-27

This will add an entry point for the ``obstacle_avoider`` node.

Go to the file ``robot_launch.py`` and replace ``def generate_launch_description():`` with:

.. literalinclude:: Code/robot_launch_sensor.py
    :language: python
    :lines: 11-47

This will create an ``obstacle_avoider`` node that will be included in the ``LaunchDescription``.

11 Test the obstacle avoidance code
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

As in task ``7``, launch the simulation from a terminal in your ROS 2 workspace:

.. tabs::

    .. group-tab:: Linux

        From a terminal in your ROS 2 workspace run:

        .. code-block:: console

            colcon build
            source install/local_setup.bash
            ros2 launch my_package robot_launch.py

    .. group-tab:: Windows

        From a terminal in your WSL ROS 2 workspace run:

        .. code-block:: console

            colcon build
            export WEBOTS_HOME=/mnt/c/Program\ Files/Webots
            source install/local_setup.bash
            ros2 launch my_package robot_launch.py

        Be sure to use the ``/mnt`` prefix in front of your path to the Webots installation folder to access the Windows file system from WSL.

    .. group-tab:: macOS

        In a terminal of the host machine (not in the container), if not done already, specify the Webots installation folder (e.g. ``/Applications/Webots.app``) and start the server using the following commands:

        .. code-block:: console

            export WEBOTS_HOME=/Applications/Webots.app
            python3 local_simulation_server.py

        Note that the server keeps running once the ROS 2 nodes are ended.
        You don't need to restart it every time you want to launch a new simulation.
        From the terminal of the Docker container, build and launch your custom package with:

        .. code-block:: console

            cd ~/ros2_ws
            colcon build
            source install/local_setup.bash
            ros2 launch my_package robot_launch.py

Your robot should go forward and before hitting the wall it should turn clockwise.
You can press ``Ctrl+F10`` in Webots or go to the ``View`` menu, ``Optional Rendering`` and ``Show DistanceSensor Rays`` to display the range of the distance sensors of the robot.

.. image:: Image/Robot_turning_clockwise.png

Summary
-------

In this tutorial, you set-up a realistic robot simulation with Webots, implemented a Python plugin to control the motors of the robot, and implemented a ROS node using the sensors to avoid the obstacles.

Next steps
----------

You might want to improve the plugin or create new nodes to change the behavior of the robot.
Taking inspiration from these previous tutorials could be a starting point:

* :doc:`../Recording-A-Bag-From-Your-Own-Node-Py`.

* :doc:`../../Intermediate/Tf2/Tf2-Main`.
