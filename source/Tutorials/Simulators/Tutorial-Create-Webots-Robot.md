Webots offers a huge number of high-quality premade robot models.
However, if you want to create a new one from scratch for your ROS 2 application then you can follow this tutorial.

You have the choice to either [build by yourself](https://github.com/cyberbotics/webots_ros2/wiki/Tutorial-Create-Webots-Robot#build-the-robot) a simple robot or directly [download it here](./assets/tutorialCreateWebotsRobot/my_world.wbt).

In case you download the world file, open it in Webots and take a look how the robot is composed. Then you can directly jump to part [Launch the driver node](https://github.com/cyberbotics/webots_ros2/wiki/Tutorial-Create-Webots-Robot#launch-the-driver-node).

# Build the robot

## Create the simulation world in Webots

Simulations in Webots rely on world files containing objects called `nodes` (different from the standard ROS node).
Before creating a robot you will create the environment in which your robot will evolve.
First, launch Webots.

Once it is open, click on `File` > `New World`.
A new empty world will be created.

![empty_world](./assets/tutorialCreateWebotsRobot/empty_world.png)

Now you will add some light, a background and an arena.
Press `Ctrl+Shift+A` to add an object or press the `Add` button (the 2nd button of the top toolbar, the `+` icon).
Then use the search bar, type `TexturedBackground`, click on the corresponding object and then press the `Add` button.

![add_background](./assets/tutorialCreateWebotsRobot/add_background.png)

Repeat the process to add a `TexturedBackgroundLight` node and a `CircleArena` node in order to obtain the same setup than in the following image.

![empty_arena](./assets/tutorialCreateWebotsRobot/empty_arena.png)

You can now navigate into your world with the `right click` of the mouse and rotate the point of view with the `left click`.
Go to `File` > `Save World As ...`, and save your file at the desired location.

## Create the robot in Webots

Your world environment is ready and it is time to create your robot!
You will start gently with a robot composed of a cylindrical body and two wheels.

### Body of the robot

Start by adding a `Robot` object.
Press the `Add` button, open `Base nodes` (double click or press the arrow button) and then click on `Robot` > `Add`.

![add_robot](./assets/tutorialCreateWebotsRobot/add_robot.png)

In the scene tree (on the left), expand the `Robot` pan.
You will now add the components of the robot.

Click on `children` > `Ctrl+Shift+A` > `Base nodes` > `Transform` > `Add` to add a `Transform` node.
Then expand the new created node (the `Transform` node) and set the translation to `0 0 0.0415` in order to elevate a bit your robot.

![transform_up](./assets/tutorialCreateWebotsRobot/transform_up.png)

Add a `Shape` node to `children` of the new node.

Select `appearance` of the `Shape` node and add a `PBRAppearance` node.
Expand the `PBRAppearance` node, click on `baseColor` and chose a color for the body of your robot, so you will be able to distinguish this part.
Also set the roughness to 1 and metalness to 0 to have a better visual aspect.

Expand the `Shape` node, select `geometry` and add a `Cylinder` node.
In the `Cylinder` node set the height to `0.08` and the radius to `0.045`.
Click on the `Cylinder` node and in the `DEF` field (under the scene tree) type `BODY`.
This mechanism of Webots will permit you to reuse the parameters of this cylinder later.

You should have the following setup at this point:

![cylinder_param](./assets/tutorialCreateWebotsRobot/cylinder_param.png)

To the `boundingObject` field of your robot add a `Transform` node.
Set also the translation of this `Tranform` to `0 0 0.0415`.
Inside this new node add to the `children` field the `BODY` node you defined previously (you will find it in the `USE` field).

![cylinder_DEF](./assets/tutorialCreateWebotsRobot/cylinder_DEF.png)

Add to your robot a `Physics` node to the corresponding field in order to enable the collision with other objects.
Then select the field `controller` > `Select...` > `<extern>` to indicate to the simulator that the controller used will be extern to Webots (as you will use a ROS node).

You should end with this setup for the second part of the body of your robot:

![cylinder_bounding](./assets/tutorialCreateWebotsRobot/cylinder_bounding.png)

### Left wheel of the robot

Now you will add the wheels to your robot.
Let's start with the left one.
Select `Robot` > `children` and then add a `HingeJoint` node.

![hinge_joint](./assets/tutorialCreateWebotsRobot/hinge_joint.png)

Then select `jointParameters` and add a `HingeJointParameters` node.
Select `device`, add a `RotationalMotor` node and then configure the `HingeJointParameters` and `RotationalMotor` nodes like below.

![left_motor](./assets/tutorialCreateWebotsRobot/left_motor.png)

`axis` will indicate around which axis the wheel should rotate and `anchor` indicates an offset.
Next select `endPoint`, add a `Solid` node and set it up like here in a similar manner you did for the body of the robot:

![left_wheel_solid](./assets/tutorialCreateWebotsRobot/left_wheel_solid.png)

Then change the name to `left wheel` and add to the `boundingObject` a `WHEEL` node (you will find it again in the `USE` field).
Add to your robot a `Physics` node to its corresponding field once again in order to enable the collisions.
You should end with the following for the `endPoint Solid` node.

![left_wheel_bounding](./assets/tutorialCreateWebotsRobot/left_wheel_bounding.png)

### Right wheel of the robot

Creating the second wheel is bit easier as you can use the `DEF-USE` mechanism of Webots.
Select `Robot` > `children` > `HingeJoint` and then add a `HingeJoint` node.
You have to select the first `HingeJoint` in order to have the second `HingeJoint` below.
This will enable to correctly benefit of the `DEF-USE` mechanism as you can only have a `USE` if there is a corresponding `DEF` defined above in the scene tree.

Once again select `jointParameters` and add a `HingeJointParameters` node.
Select `device`, add a `RotationalMotor` node and then configure the `HingeJointParameters` add `RotationalMotor` nodes like below.

![right_motor](./assets/tutorialCreateWebotsRobot/right_motor.png)

Select `endPoint`, add a `Solid` node and inside the new `children` add the `WHEEL` node.
Then change the name to `right wheel` and add to the `boundingObject` another `WHEEL` node.
Finally add to your robot a `Physics` node to its corresponding field.

![right_wheel](./assets/tutorialCreateWebotsRobot/right_wheel.png)

Congratulations!
You have now a simulation with a simple robot that can move around in the arena.
Do not forget to save your file with `Ctrl+S`.

![robot_body_wheel](./assets/tutorialCreateWebotsRobot/robot_body_wheel.png)

## Add sensors to your robot

Your robot is now able to move but it will not be able to avoid obstacle.

Let's add some sensors in order to make the robot able to detect front obstacles.
Start by adding a `DistanceSensor` node in the `children` field of your robot.
Then configure the sensor as described in the following image in order to correctly place and model the sensor:

![Sensor_1_1](./assets/tutorialCreateWebotsRobot/Sensor_1_1.png)

Next rename the sensor and configure the `lookupTable`, the `numberOfRays` and the `aperture` field.
You might need to press `lookupTable` > `Ctrl+Shift+A` in order to add a new line in the `lookupTable` field.

![Sensor_1_2](./assets/tutorialCreateWebotsRobot/Sensor_1_2.png)

Creating the second sensor is straightforward and you can follow this structure:

![Sensor_2](./assets/tutorialCreateWebotsRobot/Sensor_2.png)

Now you can confirm that your sensors are well modeled by going to `View` > `Optional Rendering` > `Show DistanceSensor Rays` or pressing `Ctrl+F10`.
You should see two white lines per sensor in a horizontal plan in front of your robot.

![Sensor_view_ray](./assets/tutorialCreateWebotsRobot/Sensor_view_ray.png)

Perfect, your robot is ready! Do not forget to save your world with `Ctrl+S`.
You can check your implementation by downloading [here](./assets/tutorialCreateWebotsRobot/my_world.wbt) the world file.

# Launch the driver node

After you ensured the new robot model is working properly you can create a ROS 2 interface for it in a few simple steps:
1. Start the simulation.
2. Execute `ros2 run webots_ros2_driver driver`.

The `driver` node will attach to the Webots robot and create a ROS 2 interface for it.

For more details on how to improve the ROS 2 interface please follow the tutorial [Creating a Custom Package](Tutorial-Creating-a-Custom-Package).
