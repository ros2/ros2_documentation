.. redirect-from::

   Concepts/About-Client-Interfaces
   Concepts/About-ROS-2-Client-Libraries
   Concepts/Basic/About-Client-Libraries

.. include:: ../../global_substitutions.txt

Client libraries
================

.. toctree::
   :maxdepth: 1
   :hidden:

   client-libraries/About-Different-Middleware-Vendors
   client-libraries/About-Executors/About-Executors
   client-libraries/About-Internal-Interfaces/About-Internal-Interfaces
   client-libraries/About-Middleware-Implementations
   client-libraries/Working-with-Client-Libraries

Client libraries are the APIs that allow users to implement their ROS 2 code.
Using client libraries, users gain access to ROS 2 concepts such as nodes, topics, services, and so on.

**[Area: Framework | Content-type: concept | Experience: beginner]**

.. contents:: Table of Contents
   :local:

Summary
-------

Client libraries are the APIs that allow users to implement their ROS code.
Client libraries expose users to the core elements of ROS including nodes and interfaces.
Nodes written using different client libraries are able to share messages with each other.

The C++ client library (``rclcpp``) and the Python client library (``rclpy``) are both client libraries which build on the fuctionality in the ROS Client Library (``rcl``).

Client libraries
----------------

Client libraries are maintained in a variety of programming languages so that users can write ROS code in the language that is best-suited for their application.
For example, you might prefer to write visualization tools in Python because it makes prototyping iterations faster, while for parts of your system that are concerned with efficiency, the nodes might be better implemented in C++.
All client libraries implement code generators which give users the capability to interact with ROS interface files in a number of supported languages.
These interface files allow nodes written using different client libraries to share messages with each other.

In addition to the language-specific communication tools, client libraries expose to users the core functionality of ROS.
For example, the following functionality can typically be accessed through a client library:

* Names and namespaces
* Time (real or simulated)
* Parameters
* Console logging
* Threading model
* Intra-process communication

ROS Client Library (``rcl``)
-----------------------------

Most of the functionality found in a client library is not specific to the programming language of the client library.
For example, the behavior of parameters and the logic of namespaces should ideally be the same across all programming languages.
Client libraries make use of a common core ROS Client Library (RCL) interface that implements logic and behavior of ROS concepts that is not language-specific.
As a result, client libraries only need to wrap the common functionality in the RCL with foreign function interfaces.
This keeps client libraries thinner and easier to develop.
For this reason, the common RCL functionality is exposed with C interfaces as the C language is typically the easiest language for client libraries to wrap.

In addition to making the client libraries lightweight, an advantage of having the common core is that the behavior between languages is more consistent.
If any changes are made to the logic/behavior of the functionality in the core RCL, such as namespaces, all client libraries that use the RCL will have these changes reflected.
Furthermore, having the common core means that maintaining multiple client libraries becomes less work when it comes to bug fixes.

The API documentation for ``rcl`` can be found `here <{package_link(rcl)}>`__.

Language-specific functionality
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Client library concepts that require language-specific features/properties are not implemented in the RCL but instead are implemented in each client library.
For example, threading models used by "spin" functions will have implementations that are specific to the language of the client library.

Related content
---------------
* :doc:`How-ROS-Works`
* :doc:`About-Nodes`
* :doc:`About-Parameters`
* :doc:`Interfaces-Topics-Services-Actions`

FAQs
----

What is a client library?
   A client library is the API you use to write ROS code in a given programming language.
   It gives you access to core ROS concepts such as nodes, topics, services, parameters, and logging.

Can nodes written with different client libraries communicate?
   Yes.
   Client libraries generate language bindings for ROS interface files, so nodes can share messages even when they use different languages, for example ``rclcpp`` and ``rclpy``.

What is the ROS Client Library (``rcl``)?
   ``rcl`` is the common C core that implements language-independent ROS behaviour, such as parameters and namespaces.
   Language-specific client libraries wrap ``rcl`` instead of reimplementing that shared logic.

Why does ROS use a common ``rcl`` core?
   Shared behaviour stays consistent across languages, and bug fixes or behaviour changes in ``rcl`` apply to every client library that uses it.
   That also keeps the language-specific libraries thinner and easier to maintain.

Is all client library behaviour implemented in ``rcl``?
   No.
   Features that depend on the programming language, such as the threading model used by spin functions, are implemented in each client library rather than in ``rcl``.
