Reporting an issue — how-to
===========================

Issue reports help the ROS community identify bugs, suggest enhancements, improve documentation, and resolve package-specific problems.
This article explains how to check whether an issue has already been reported and what information to provide in a new issue.
With this information, you can provide clear, complete issue details in the right ROS repository.

**Area: community | Content-type: how-to | Experience: beginner, intermediate, expert**

.. contents:: Table of Contents
   :depth: 2
   :local:

Summary
-------

* **Documentation issue tracker**: `ROS documentation repository <https://github.com/ros2/ros2_documentation/issues>`__.
* **Code issues lists**: in a repository under the `ROS organisation <https://github.com/orgs/ros2/repositories>`__.
* **Catch-all issues list**: `top level ROS repository <https://github.com/ros2/ros2/issues>`__.

Prerequisites
-------------

There are no prerequisites.

Steps
-----

1 Checking the issue
^^^^^^^^^^^^^^^^^^^^

If you identify bugs, have suggestions for enhancements, or a question specific to one package, you can open an issue on GitHub.
For example, if you are following the :doc:`/Tutorials` and come across an instruction that doesn't work on your system, you can open an issue in the `ROS documentation <https://github.com/ros2/ros2_documentation>`__ repo.

You can search for individual ROS repositories on `ROS GitHub <https://github.com/ros2>`__.

Before opening an issue:

#. Check if other users have reported similar issues by searching across the ``ros2`` and ``ament`` GitHub organizations using, for example, this search query: `<https://github.com/search?q=user%3Aros2+user%3Aament+turtlesim&type=Issues>`__.
#. Check the `Robotics Stack Exchange <https://robotics.stackexchange.com/>`__ to see if someone else has already reported your issue.

If your issue has not been reported, you can open an issue in the appropriate repository's issue tracker.
If it's not clear which tracker to use for a particular issue, create it in the `ros2/ros2 repository <https://github.com/ros2/ros2/issues>`__ and we'll have a look at it.

2 Reporting the issue
^^^^^^^^^^^^^^^^^^^^^

When reporting an issue, use the following guidelines to make sure you include enough information for another person to understand and (where relevant) reproduce the issue.
It's particularly helpful if you test for your issue with as many alternatives as you can in each category.

Required information
""""""""""""""""""""

#. Use a descriptive **title** for the issue.

   **Bad**: "rviz doesn't work".

   **Good**: "Rviz crashing looking for missing ``.so`` after latest apt update"

#. Add steps to the issue **description** describing exactly how to reproduce the issue.
   If you are following a ROS tutorial or online instructions, provide a link to those specific instructions.

   Issues are more likely to be resolved if others can reproduce them easily.

#. Include the following information:

   * **Operating system and version**

     ROS supports multiple platforms, and some bugs are specific to particular versions of operating systems/compilers.

   * **Installation method**

     Some issues only manifest if ROS has been installed from binary archives or from deb packages.
     This can help us determine if the issue is with the packaging process.

   * **Specific version of ROS**

     Some bugs may be present in a particular ROS release, but fixed in a later release.
     It is important to know if your installation includes these fixes.

   * **DDS/RMW implementation** (see `this page </Concepts/Intermediate/About-Different-Middleware-Vendors>` for how to determine which one).

     Communication issues may be specific to the underlying ROS middleware being used.

   * **ROS client library**

     This helps us narrow down the layer in the stack where the issue might be.

#. Include any warnings or errors which are displayed as part of the issue.

   Cut and paste the warnings or errors directly from the terminal window to which they were printed.
   Please do not re-type the warnings or errors or use screenshots of the terminal.

Useful additional information
"""""""""""""""""""""""""""""

* If you are reporting a bug, consider providing a `short, self contained, correct (compilable) example <https://sscce.org/>`__.
* When discussing any compiling/linking/installation issues, provide the version number of your compiler.
* If relevant to your issue, you can also include your:

  * ROS environment variables (``env | grep ROS``)
  * Backtraces
  * Relevant config files
  * Graphics card model and driver version
  * ``Ogre.log`` for rviz, if possible (run with ``rviz -l``)
  * Bag files and code samples which reproduce the problem
  * GIFs or video clips to demonstrate the problem

* Also describe any troubleshooting which you have already attempted, for example:

  * Upgrading to the latest version of the code, which may include bug fixes that have not been released yet.

    For more information, see :ref:`building-from-source` and follow the instructions to get the ``rolling`` branch.

  * Trying to reproduce your issue with a different RMW implementation.

    For more information, see :doc:`../../../How-To-Guides/Working-with-multiple-RMW-implementations`.

Related content
---------------

* :doc:`../../Contributing`
* :doc:`Triaging-an-issue`

FAQs
----

TBC
