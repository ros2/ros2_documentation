Monitoring for parameter changes (C++)
======================================

**Goal:** Learn to use the ParameterEventsSubscriber class to monitor for parameter changes

**Tutorial level:** Beginner

**Time:** 20 minutes

.. contents:: Contents
   :depth: 2
   :local:

Background
----------

Often a node needs to respond to changes to its own parameters or another node's parameters. The ParameterEventsSubscriber class makes it easy to listen for these kinds of parameter changes so that your code can respond to them. This tutorial will show you how to use the C++ version of the ParameterEventsSubscriber class to monitor for changes to a node's own parameters as well as changes to another node's parameters.

Prerequisites
-------------

Before starting this tutorial, you should first complete the following tutorials: 

- :ref:`ROS2Params`
- :ref:`CppParamNode`

Tasks
-----

The ParameterEventsSubscriber class provides function interfaces to handle parameter events.

1 Create a package
^^^^^^^^^^^^^^^^^^

Open a new terminal and :ref:`source your ROS 2 installation <ConfigROS2>` so that ``ros2`` commands will work.

Navigate into the ``dev_ws`` directory created in a previous tutorial (or follow `these instructions <https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/#create-a-new-directory>`_ if you no longer have the directory and need to create it again).

Recall that packages should be created in the ``src`` directory, not the root of the workspace. So, navigate into ``dev_ws/src`` and then create a new package there:

.. code-block:: console

  ros2 pkg create --build-type ament_cmake cpp_parameter_events_subscriber --dependencies rclcpp

Your terminal will return a message verifying the creation of your package ``cpp_parameter_events_subscriber`` and all its necessary files and folders.

The ``--dependencies`` argument will automatically add the necessary dependency lines to ``package.xml`` and ``CMakeLists.txt``.

1.1 Update ``package.xml``
~~~~~~~~~~~~~~~~~~~~~~~~~~

Because you used the ``--dependencies`` option during package creation, you don’t have to manually add dependencies to ``package.xml`` or ``CMakeLists.txt``. As always, though, make sure to add the description, maintainer email and name, and license information to ``package.xml``.

.. code-block:: xml

  <description>C++ parameter events client tutorial</description>
  <maintainer email="you@email.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

2 Write the C++ node
^^^^^^^^^^^^^^^^^^^^

Inside the ``dev_ws/src/cpp_parameter_events_subscriber/src`` directory, create a new file called ``parameter_events_subscriber.cpp`` and paste the following code within:

.. code-block:: C++

    #include <memory>
    
    #include "rclcpp/rclcpp.hpp"
    
    class SampleNodeWithParameters : public rclcpp::Node
    {
    public:
      SampleNodeWithParameters()
      : Node("node_with_parameters")
      {
        this->declare_parameter("an_int_param", 0);
    
        // Create a parameter subscriber that can be used to monitor parameter changes
        // (for this node's parameters as well as other nodes' parameters)
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventsSubscriber>(this);
    
        // Set a callback for this node's integer parameter, "an_int_param"
        auto cb = [this](const rclcpp::Parameter & p) {
            RCLCPP_INFO(
              this->get_logger(), "cb: Received an update to parameter \"%s\" of type %s: \"%ld\"",
              p.get_name().c_str(),
              p.get_type_name().c_str(),
              p.as_int());
          };
        cb_handle_ = param_subscriber_->add_parameter_callback("an_int_param", cb);
      }
    
      ~SampleNodeWithParameters()
      {
        param_subscriber_->remove_parameter_callback(cb_handle_.get());
      }
    
    private:
      std::shared_ptr<rclcpp::ParameterEventsSubscriber> param_subscriber_;
      std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
    };
    
    int main(int argc, char ** argv)
    {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<SampleNodeWithParameters>());
      rclcpp::shutdown();
    
      return 0;
    }

2.1 Examine the code
~~~~~~~~~~~~~~~~~~~~
The first statement, ``#include <memory>`` is included so that the code can utilize the std::make_shared template. The next, ``#include "rclcpp/rclcpp.hpp"`` is included to allow the code to reference the various functionality provided by the rclcpp interface, including the ParameterEventsSubscriber class. 

After the class declaration, the code defines a class, ``SampleNodeWithParameters``. The constructor for the class, declares an integer parameter ``an_int_param``, with a default value of 0. Next, the code creates a ``ParameterEventSubscriber`` that will be used to monitor changes to parameters. Finally, the code creates a lambda function and sets it as the callback to invoke whenever ``an_int_param`` is updated. 

.. code-block:: C++

    SampleNodeWithParameters()
    : Node("node_with_parameters")
    {
      this->declare_parameter("an_int_param", 0);
  
      // Create a parameter subscriber that can be used to monitor parameter changes
      // (for this node's parameters as well as other nodes' parameters)
      param_subscriber_ = std::make_shared<rclcpp::ParameterEventsSubscriber>(this);
  
      // Set a callback for this node's integer parameter, "an_int_param"
      auto cb = [this](const rclcpp::Parameter & p) {
          RCLCPP_INFO(
            this->get_logger(), "cb: Received an update to parameter \"%s\" of type %s: \"%ld\"",
            p.get_name().c_str(),
            p.get_type_name().c_str(),
            p.as_int());
        };
      cb_handle_ = param_subscriber_->add_parameter_callback("an_int_param", cb);
    }
  
The ``ParameterEventSubscriber``'s add_parameter_callback method returns a callback handle that is stored in a member variable. This handle is used in the ``~SampleNodeWithParameters`` destructor to remove the callback when the node is destroyed.

.. code-block:: C++

    ~SampleNodeWithParameters()
    {
      param_subscriber_->remove_parameter_callback(cb_handle_.get());
    }

Following the ``SampleNodeWithParameters`` is a typical ``main`` function which initializes ROS, spins the sample node so that it can send and receive messages, and then shuts down after the user enters ^C at the console.

.. code-block:: C++

    int main(int argc, char** argv)
    {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<SampleNodeWithParameters>());
      rclcpp::shutdown();

      return 0;
    }


2.2 Add executable
~~~~~~~~~~~~~~~~~~

To build this code, first open the ``CMakeLists.txt`` file and add the following lines of code below the dependency ``find_package(rclcpp REQUIRED)`` 

.. code-block:: console

    add_executable(parameter_events_subscriber src/parameter_events_subscriber.cpp)
    ament_target_dependencies(parameter_events_subscriber rclcpp)

    install(TARGETS
      parameter_events_subscriber
      DESTINATION lib/${PROJECT_NAME}
    )

3 Build and run
^^^^^^^^^^^^^^^

It's good practice to run ``rosdep`` in the root of your workspace (``dev_ws``) to check for missing dependencies before building:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        rosdep install -i --from-path src --rosdistro <distro> -y

   .. group-tab:: macOS

      rosdep only runs on Linux, so you can skip ahead to next step.

   .. group-tab:: Windows

      rosdep only runs on Linux, so you can skip ahead to next step.

Navigate back to the root of your workspace, ``dev_ws``, and build your new package:

.. code-block:: console

    colcon build --packages-select cpp_parameter_events_subscriber

Open a new terminal, navigate to ``dev_ws``, and source the setup files:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      call install/setup.bat

Now run the node:

.. code-block:: console

     ros2 run cpp_parameter_events_subscriber parameter_events_subscriber

The node is now active and has a single parameter and will print a message whenever this parameter is updated. To test this, open up another terminal and source the ROS setup file as before (. install/setup.bash) and execute the following command:

.. code-block:: console

    ros2 param set node_with_parameters an_int_param 43

The terminal running the node will display a message similar to the following:

.. code-block:: console

    [INFO] [1606950498.422461764] [node_with_parameters]: cb: Received an update to parameter "an_int_param" of type integer: "43"

The callback we set previously in the node has been invoked and has displayed the new updated value. You can now terminate the running parameter_events_subscriber sample using ^C in the terminal.

3.1 Monitor changes to another node's parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can also use the ParameterEventsSubscriber to monitor parameter changes to another node's parameters. Let's update the SampleNodeWithParameters class to also monitor for changes to a parameter in another node.  We will use the parameter_blackboard demo application to host a double parameter that we will monitor for updates. 

First update the constructor to add the following code after the existing code:

.. code-block:: C++

    // Now, add a callback to monitor any changes to the remote node's parameter. In this
    // case, we supply the remote node name.
    auto cb2 = [this](const rclcpp::Parameter & p) {
        RCLCPP_INFO(
          this->get_logger(), "cb2: Received an update to parameter \"%s\" of type: %s: \"%.02lf\"",
          p.get_name().c_str(),
          p.get_type_name().c_str(),
          p.as_double());
      };
    auto remote_node_name = std::string("parameter_blackboard");
    auto remote_param_name = std::string("a_double_param");
    cb_handle2_ = param_subscriber_->add_parameter_callback(remote_param_name, cb2, remote_node_name);


The destructor and member variables need to be updated as well to account for the new ``cb_handle2`` member variable:

.. code-block:: C++

    ~SampleNodeWithParameters()
    {
      param_subscriber_->remove_parameter_callback(cb_handle_.get());
      param_subscriber_->remove_parameter_callback(cb_handle2_.get());  // Add this
    }

  private:
    std::shared_ptr<rclcpp::ParameterEventsSubscriber> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle2_;  // Add this
  };


In a terminal, navigate back to the root of your workspace, ``dev_ws``, and build your updated package as before:

.. code-block:: console

    colcon build --packages-select cpp_parameter_events_subscriber

Then source the setup files:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: macOS

    .. code-block:: console

      . install/setup.bash

  .. group-tab:: Windows

    .. code-block:: console

      call install/setup.bat

Now, to test monitoring of remote parameters, first run the newly-built parameter_events_subscriber code:

.. code-block:: console

     ros2 run cpp_parameter_events_subscriber parameter_events_subscriber

Next, from another teminal (with ROS initialized), run the parameter_blackboard demo application, as follows:

.. code-block:: console

     ros2 run demo_nodes_cpp parameter_blackboard

Finally, from a third terminal (with ROS initialized), let's set a parameter on the parameter_blackboard node:

.. code-block:: console

     ros2 param set parameter_blackboard a_double_param 3.45

Upon executing this command, you should see output in the parameter_events_subscriber window, indicating that the callback function was invoked upon the parameter update:

.. code-block:: console

    [INFO] [1606952588.237531933] [node_with_parameters]: cb2: Received an update to parameter "a_double_param" of type: double: "3.45"

Summary
-------

You created a node with a parameter and used the ParameterEventsSubscriber class to set a callback to monitor changes to that parameter. You also used the same class to monitor changes to a remote node. The ParameterEventsSubscriber is a convenient way to monitor for parameter changes so that you can then respond to the updated values.

Next steps
----------

Now that you have examined parameters and have some packages and ROS 2 systems of your own, the :ref:`next tutorial <Ros2Doctor>` will show you how to examine issues in your environment and systems in case you have problems.
