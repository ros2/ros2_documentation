.. _ROS-2-Client-Libraries:

About ROS 2 client libraries
============================

.. contents:: Table of Contents
   :local:

Overview
--------

Client libraries are the APIs that allow users to implement their ROS code.
They are what users use to get access to ROS concepts such as nodes, topics, services, etc.
Client libraries come in a variety of programming languages so that users may write ROS code in the language that is best-suited for their application.
For example, you might prefer to write visualization tools in Python because it makes prototyping iterations faster, while for parts of your system that are concerned with efficiency, the nodes might be better implemented in C++.

Nodes written using different client libraries are able to share messages with each other because all client libraries implement code generators that provide users with the capability to interact with ROS interface files in the respective language.

In addition to the language-specific communication tools, client libraries expose to users the core functionality that makes ROS “ROS”.
For example, here is a list of functionality that can typically be accessed through a client library:


* Names and namespaces
* Time (real or simulated)
* Parameters
* Console logging
* Threading model
* Intra-process communication

Supported client libraries
--------------------------

The C++ client library (``rclcpp``) and the Python client library (``rclpy``) are both client libraries which utilize common functionality in the RCL.

While the C++ and Python client libraries are maintained by the core ROS 2 team, members of the ROS 2 community have created additional client libraries:


* `JVM and Android <https://github.com/esteve/ros2_java>`__
* `Objective C and iOS <https://github.com/esteve/ros2_objc>`__
* `C# <https://github.com/firesurfer/rclcs>`__
* `Swift <https://github.com/younata/rclSwift>`__
* `Node.js <https://www.npmjs.com/package/rclnodejs>`__
* `Ada <https://github.com/ada-ros/ada4ros2>`__
* `_.NET Core, UWP and C# <https://github.com/esteve/ros2_dotnet>`__
* `Rust <https://github.com/ros2-rust/ros2_rust>`__

Common functionality: the RCL
-----------------------------

Most of the functionality found in a client library is not specific to the programming language of the client library.
For example, the behavior of parameters and the logic of namespaces should ideally be the same across all programming languages.
Because of this, rather than implementing the common functionality from scratch, client libraries make use of a common core ROS Client Library (RCL) interface that implements logic and behavior of ROS concepts that is not language-specific.
As a result, client libraries only need to wrap the common functionality in the RCL with foreign function interfaces.
This keeps client libraries thinner and easier to develop.
For this reason the common RCL functionality is exposed with C interfaces as the C language is typically the easiest language for client libraries to wrap.

In addition to making the client libraries light-weight, an advantage of having the common core is that the behavior between languages is more consistent.
If any changes are made to the logic/behavior of the functionality in the core RCL -- namespaces, for example -- all client libraries that use the RCL will have these changes reflected.
Furthermore, having the common core means that maintaining multiple client libraries becomes less work when it comes to bug fixes.

`The API documentation for the RCL can be found here. <https://docs.ros2.org/latest/api/rcl/>`__

Language-specific functionality
-------------------------------

Client library concepts that require language-specific features/properties are not implemented in the RCL but instead are implemented in each client library.
For example, threading models used by “spin” functions will have implementations that are specific to the language of the client library.

Demo
----

For a walkthrough of the message exchange between a publisher using ``rclpy`` and a subscriber using ``rclcpp``\ , we encourage you to watch `this ROSCon talk <https://vimeo.com/187696091>`__ starting at 17:25 `(here are the slides) <https://roscon.ros.org/2016/presentations/ROSCon%202016%20-%20ROS%202%20Update.pdf>`__.

Comparison to ROS 1
-------------------

In ROS 1, all client libraries are developed "from the ground up".
This allows for the ROS 1 Python client library to be implemented purely in Python, for example, which brings benefits of such as not needing to compile code.
However, naming conventions and behaviors are not always consistent between client libraries, bug fixes have to be done in multiple places, and there is a lot of functionality that has only ever been implemented in one client library (e.g. UDPROS).

Summary
-------

By utilizing the common core ROS client library, client libraries written in a variety of programming languages are easier to write and have more consistent behavior.
