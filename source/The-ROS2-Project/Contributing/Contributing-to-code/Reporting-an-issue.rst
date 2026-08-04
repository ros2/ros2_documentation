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

* **Code issue trackers**: in a repository under the `ROS organisation <https://github.com/orgs/ros2/repositories>`__ or elsewhere on GitHub.
* **Documentation issue tracker**: `ROS documentation repository <https://github.com/ros2/ros2_documentation/issues>`__.
* **Catch-all issue tracker**: `top level ROS repository <https://github.com/ros2/ros2/issues>`__.

Prerequisites
-------------

There are no prerequisites.

Steps
-----

1 Identifying where to report the issue
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you identify a bug, have a suggestion for enhancement, or a question specific to one package, you can report an issue on GitHub.
For example, if you are following a :doc:`tutorial </First-Steps>` and come across an instruction that doesn't work on your system, you can report an issue in the `ROS documentation repository <https://github.com/ros2/ros2_documentation/issues>`__.

You can report code issues in the ROS repository for the package where the error occurred.
This is generally the last file listed in the stack trace.

You can search for ROS packages in the following locations:

* Core ROS 2 packages, found in the `ROS organisation <https://github.com/ros2>`__
* ROS binary packages, found on the `ROS Index <index.ros.org>`__, which point to the appropriate GitHub repository.
* Source ROS packages which are located elsewhere on GitHub.

If it's not clear which issue tracker to use for a particular issue, report the issue in the `top-level ROS repository <https://github.com/ros2/ros2/issues>`__ and we'll have a look at it.

2 Checking the issue
^^^^^^^^^^^^^^^^^^^^

Before opening an issue:

#. Check if other users have reported similar issues by searching across the ``ros2`` and ``ament`` GitHub organizations using, for example, this `search query <https://github.com/search?q=user%3Aros2+user%3Aament+turtlesim&type=Issues>`__.
#. Check the `Robotics Stack Exchange <https://robotics.stackexchange.com/>`__ to see if someone else has already reported your issue.

If your issue has not been reported, you can open an issue in the issue tracker of the repository you have identified.

3 Reporting the issue
^^^^^^^^^^^^^^^^^^^^^

When reporting an issue, use the following steps to make sure you include enough information for another person to understand and (where relevant) reproduce the issue.

#. In the issue tracker of your selected repository, select **New issue**.
#. From the options shown, select the issue type which is most appropriate for the issue you are raising:

   * If you are in a ROS code repository or the top-level ROS repository, select **Bug report** or **Feature request**.
   * If you are in the ROS documentation repository, select **Documentation issue**.
   * Alternatively, if the specific ROS repository you are in has a unique issue template, you can use that instead.

#. Add a descriptive **Title** for the issue.

   **Bad**: "rviz doesn't work".

   **Good**: "Rviz crashing looking for missing ``.so`` after latest apt update"

#. Complete the remaining fields in the issue template, following the prompt text shown and the formatting of any pre-filled fields.

Additional information
""""""""""""""""""""""

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
  * Log files (see :doc:`/ROS-Framework/nodes/About-Logging/About-Logging`) or logs from the RQT console (see :doc:`/ROS-Framework/nodes/Working-with-nodes/Using-Rqt-Console/Using-Rqt-Console`)

* Also describe any troubleshooting which you have already attempted, for example:

  * Upgrading to the latest version of the code, which may include bug fixes that have not been released yet.

    For more information, see :ref:`building-from-source` and follow the instructions to get the ``rolling`` branch.

  * Trying to reproduce your issue with a different RMW implementation.

    For more information, see :doc:`/Get-Started/Installation/RMW-Implementations/Working-with-multiple-RMW-implementations`.

Related content
---------------

* :doc:`../../Contributing`
* :doc:`Triaging-an-issue`

FAQs
----

TBC
