Creating an Action
==================

In this tutorial we look how to define an action in a ROS package.

Make sure you have satisfied all `prerequisites <../Actions>`.

Defining an Action
------------------

Just like in ROS 1, actions are defined in ``.action`` files of the form:

.. code-block:: bash

    # Request
    ---
    # Result
    ---
    # Feedback

An action definition is made up of three message definitions separated by ``---``.
An instance of an action is typically referred to as a *goal*.
A *request* message is sent from an action client to an action server initiating a new goal.
A *result* message is sent from an action server to an action client when a goal is done.
*Feedback* messages are periodically sent from an action server to an action client with updates about a goal.

Say we want to define a new action "Fibonacci" for computing the `Fibonacci sequence <https://en.wikipedia.org/wiki/Fibonacci_number>`__.

First, create a directory ``action`` in our ROS package.
With your favorite editor, add the file ``action/Fibonacci.action`` with the following content:

.. code-block:: bash

    int32 order
    ---
    int32[] sequence
    ---
    int32[] partial_sequence

The goal request is the ``order`` of the Fibonacci sequence we want to compute, the result is the final ``sequence``, and the feedback is the ``partial_sequence`` computed so far.

Building an Action
------------------

Before we can use the new Fibonacci action type in our code, we must pass the definition to the rosidl code generation pipeline.
This is accomplished by adding the following lines to our ``CMakeLists.txt``:

.. code-block:: cmake

    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
      "action/Fibonacci.action"
    )

We should also add the required dependencies to our ``package.xml``:

.. code-block:: xml

    <buildtool_depend>rosidl_default_generators</buildtool_depend>

    <depend>action_msgs</depend>

    <member_of_group>rosidl_interface_packages</member_of_group>

Note, we need to depend on ``action_msgs`` since action definitions include additional metadata (e.g. goal IDs).

We should now be able to build the package containing the "Fibonacci" action definition:

.. code-block:: bash

    # Change to the root of the workspace (ie. action_ws)
    cd ../..
    # Build
    colcon build

We're done!

By convention, action types will be prefixed by their package name and the word ``action``.
So when we want to refer to our new action, it will have the full name ``action_tutorials/action/Fibonacci``.

We can check that our action built successfully with the command line tool:

.. tabs::

  .. group-tab:: Dashing

    .. code-block:: bash

       # Source our workspace
       # On Windows: call install/setup.bat
       . install/setup.bash
       # Check that our action definition exists
       ros2 action show action_tutorials/action/Fibonacci

  .. group-tab:: Eloquent and newer

    .. code-block:: bash

       # Source our workspace
       # On Windows: call install/setup.bat
       . install/setup.bash
       # Check that our action definition exists
       ros2 interface show action_tutorials/action/Fibonacci


You should see the Fibonacci action definition printed to the screen.
