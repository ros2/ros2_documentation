.. _Simulators:

Setting-up a Robot Simulation (Webots)
======================================

**Goal:** Setup a robot simulation and control it from ROS 2.

**Tutorial level:** Advanced

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

Several robot simulators can be used with ROS 2, such as Gazebo, Ignition, Webots, etc.
Unlike turtlesim, they provide fairly realistic results relying on physics-based models for robots, sensors, actuators and objects.
Hence, what you observe in simulation is very close to what you will get when transferring your ROS 2 controllers to a real robot.
In this tutorial, you are going to use the Webots robot simulator to set-up and run a very simple ROS 2 simulation scenario.

The ``webots_ros2`` package provides an interface between ROS 2 and Webots.
It includes several sub-packages, but in this tutorial, you are going to use only the ``webots_ros2_driver`` sub-package to implement a Python plugin controlling a simulated robot.
Some other sub-packages contain demos with different robots such as the TurtleBot3.
They are documented in the :doc:`../../Tutorials` page under the Demos section.

Prerequisites
-------------

It is recommended to understand basic ROS principles covered in the beginner :doc:`../../Tutorials`.
In particular, :doc:`../Turtlesim/Introducing-Turtlesim`, :doc:`../Topics/Understanding-ROS2-Topics`, :doc:`../Workspace/Creating-A-Workspace`, :doc:`../Creating-Your-First-ROS2-Package` and :doc:`../Launch-Files/Creating-Launch-Files` are useful prerequisites.
Finally, you will need to install ``webots_ros2`` with this command:

.. code-block:: bash

        sudo apt-get install ros-{DISTRO}-webots-ros2

Tasks
-----

1 Create the package structure
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Let's organize the code in a custom ROS 2 package.
Create a new package named ``my_package`` from the ``src`` folder of your ROS 2 workspace.

.. code-block:: console

        cd ~/ros2_ws/src
        ros2 pkg create --build-type ament_python --node-name my_robot_driver my_package

The ``--node-name my_robot_driver`` option should create a ``my_robot_driver.py`` template Python plugin in the ``my_package`` subfolder that you will modify later.
Let's add a ``launch`` and a ``worlds`` folder inside the ``my_package`` folder.

.. code-block:: console

        cd my_package
        mkdir launch worlds

You should end with the following folder structure:

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


3 Prepare the package.xml file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Open the ``package.xml`` file in your favorite editor and add the following packages inside the ``<package format="3">`` tag:

.. literalinclude:: Code/package.xml
    :language: xml
    :lines: 10-12

These packages will be needed by the ``my_robot_driver.py`` plugin.

4 Change the my_robot_driver.py file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Open ``my_package/my_robot_driver.py`` in your favorite editor and replace its contents with the following:

.. literalinclude:: Code/my_robot_driver.py
    :language: python

As you can see, the ``MyRobotDriver`` class implements three methods.

The first method, named ``init(self, ...)``, is actually the ROS node counterpart of the Python ``__init__(self, ...)`` constructor.
It first gets the robot instance from the simulation (which can be used to access the `Webots robot API <https://cyberbotics.com/doc/reference/robot?tab-language=python>`_).
Then, it gets the two motor instances and initialize them with a target position and a target velocity.
Finally a ROS node is created and a callback method is registered for a ROS topic named ``/cmd_vel`` that will handle ``Twist`` messages.

.. literalinclude:: Code/my_robot_driver.py
    :language: python
    :lines: 11-27

Then comes the implementation of the ``__cmd_vel_callback(self, twist)`` callback private method that will be called for each ``Twist`` message received on the ``/cmd_vel`` topic.

.. literalinclude:: Code/my_robot_driver.py
    :language: python
    :lines: 29-30

Finally, the ``step(self)`` method is called at every time step of the simulation.
The call to ``rclpy.spin_once()`` is needed to keep the ROS node running smoothly.
At each time step, if ``self.__target_twist`` is not null, motors commands will be computed and applied.
If ``linear.x`` is negative, the robot will turn in place.
Otherwise it will go forward and turn in case ``linear.y`` is not null.

.. literalinclude:: Code/my_robot_driver.py
    :language: python
    :lines: 32-53

.. note::

    The purpose of this tutorial is to show a basic example with a minimum number of dependencies.
    However, you could avoid the use of a python plugin by using another ``webots_ros2`` sub-package named ``webots_ros2_control`` that facilitates the control of a differential wheeled robot, but introduces a new dependency.

5 Create the my_robot_webots.urdf file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You now have to create a URDF file to declare the ``my_robot_driver.py`` Python plugin.
This will allow the ``webots_ros2_driver`` ROS node to launch the plugin.

In the ``my_package/resource`` folder create a text file named ``my_robot_webots.urdf`` with this contents:

.. literalinclude:: Code/my_robot_webots.urdf
    :language: xml

.. note::

    This simple URDF file doesn't contain any link or joint information about the robot as it is not needed in this tutorial.
    However, URDF files usually contain much more information as explained in the :doc:`../URDF/URDF-Main`.

6 Create the launch file
^^^^^^^^^^^^^^^^^^^^^^^^

Let's create now the launch file to easily launch the simulation and your ROS controller with a single command.
In the ``my_package/launch`` folder create a new text file named ``robot_launch.py`` with this code:

.. literalinclude:: Code/robot_launch.py
    :language: python

The ``WebotsLauncher`` object is a custom action that allows you to start a Webots simulation instance.
You have to specify in the constructor which world file the simulator will open.

.. literalinclude:: Code/robot_launch.py
    :language: python
    :lines: 14-16

Then, the ROS node interacting with the simulated robot is created.
This node, named ``driver``, is located in the ``webots_ros2_driver`` package.
In your case, you need to run a single instance of this node, because you have a single robot in the simulation.
But if you had more robots in the simulation, you would have to run one instance of this node per robot.
The ``robot_description`` parameter holds the contents of the URDF file which refers to the ``my_robot_driver.py`` Python plugin.

.. literalinclude:: Code/robot_launch.py
    :language: python
    :lines: 18-25

Finally, the two nodes are started in such a way that if the ``webots`` launcher node is shutdown, the other ``my_robot_driver`` node will be shutdown as well.
And closing Webots from the graphical user interface will automatically shutdown the ``webots`` launcher and hence the ``my_robot_driver`` node.

.. literalinclude:: Code/robot_launch.py
    :language: python
    :lines: 27-36

7 Modify the setup.py file
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Finally before you can start the launch file, you have to modify the setup.py file to include the extra files you added.
Open ``my_package/setup.py`` and replace its contents with:

.. literalinclude:: Code/setup.py
    :language: python

This sets-up the package and adds in the ``data_files`` variable the newly added files: ``my_world.wbt``, ``my_robot_webots.urdf`` and ``robot_launch.py``.

8 Test the code
^^^^^^^^^^^^^^^

From a terminal in your ROS2 workspace run:

.. code-block:: bash

            colcon build
            source install/local_setup.bash
            ros2 launch my_package robot_launch.py

This will launch the simulation. 
Webots will be automatically installed on the first run in case it was not already installed.

.. note::

    If you want to install Webots manually, you can download it `here <https://github.com/cyberbotics/webots/releases/latest>`_.


Then, open a second terminal and send a command with:

.. code-block:: bash

            ros2 topic pub /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

Your robot is now moving forward!

.. image:: Image/Step_25.png

At this point, the robot is now able to blindly follow your motor commands.
But it will eventually bump into the wall as you order it to move forwards.

.. image:: Image/Step_26.png

To prevent this, let's use the sensors of the robot to detect the obstacles and avoid them.

9 Updating package.xml and my_robot_webots.urdf
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Let's modify these two files to enable the sensors.
Open ``package.xml`` and add the following content inside the ``<package format="3">`` tag:

.. literalinclude:: Code/package_sensor.xml
    :language: xml
    :lines: 11

Then, in the ``my_robot_webots.urdf`` file, add the following content inside the ``<webots>`` tag:

.. literalinclude:: Code/my_robot_webots_sensor.urdf
    :language: xml
    :lines: 4-18

The ROS 2 interface uses the standard parameters in the ``<ros>`` tags to enable the **DistanceSensor** nodes and name their topics.

10 Creating a ROS node to avoid obstacles
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The robot will use a standard ROS node to detect the wall and send motor commands to avoid it.
In the ``my_package/my_package/`` folder, create a file named ``obstacle_avoider.py`` with this code:

.. literalinclude:: Code/obstacle_avoider.py
    :language: python

This node will create a publisher for the command and subscribe to the sensors topics here:

.. literalinclude:: Code/obstacle_avoider.py
    :language: python
    :lines: 14-17

When a measure is received from the left sensor it will be copied to a member field:

.. literalinclude:: Code/obstacle_avoider.py
    :language: python
    :lines: 19-20

Finally, a message will be sent to the ``/cmd_vel`` topic in case any of the two sensors has detected an obstacle.
If the right sensor detects an obstacle, ``command_message`` will make the robot turn clockwise in place.
Otherwise, if only the left sensor detects an obstacle, ``command_message`` will make the robot move forward, leaning to the right.

.. literalinclude:: Code/obstacle_avoider.py
    :language: python
    :lines: 22-37

11 Updating setup.py and robot_launch.py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You have to modify these two other files to launch your new node.
Go to the file ``setup.py`` and replace ``'console_scripts'`` with:

.. literalinclude:: Code/setup_sensor.py
    :language: python
    :lines: 24-26

This will add an entry point for the ``obstacle_avoider`` node.

Go to the file ``robot_launch.py`` and replace ``def generate_launch_description():`` with:

.. literalinclude:: Code/robot_launch_sensor.py
    :language: python
    :lines: 10-42

This will create an ``obstacle_avoider`` node that will be included in the ``LaunchDescription``.

12 Test the obstacle avoidance code
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Repeat the same commands as in tasks ``8`` to test your code.
From a terminal in your ROS 2 workspace run:

.. code-block:: bash

            colcon build
            source install/local_setup.bash
            ros2 launch my_package robot_launch.py

Then, open a second terminal and run:

.. code-block:: bash

            ros2 topic pub /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

Your robot will go forward and before hitting the wall it will turn clockwise.

.. image:: Image/Robot_clock_wise.png

Summary
-------

In this tutorial, you set-up a realistic robot simulation with Webots, implemented a Python plugin to control the motors of the robot, and implemented a ROS node using the sensors to avoid the obstacles.

Next steps
----------

You might want to improve the plugin or create new nodes to change the behavior of the robot.
Taking inspiration from these previous tutorials is a good starting point:

* :doc:`../Ros2bag/Recording-A-Bag-From-Your-Own-Node-Python`.

* :doc:`../Tf2/Tf2-Main`.
