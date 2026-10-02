Triaging an issue — how-to
==========================

Issue triage helps ROS contributors turn incoming bug reports and enhancement requests into clear, actionable work.
This article explains how to find, check, reproduce, assign, and label issues in ROS repositories.
With this information, you'll be able to route issues to the right place, with correct information, and prepare them for work by developers.

**Area: community | Content-type: how-to | Experience: beginner, intermediate, expert**

.. contents:: Table of Contents
   :depth: 2
   :local:

Summary
-------

We welcome anyone to triage issues.

* **Code issue trackers**: in a repository under the `ROS organisation <https://github.com/orgs/ros2/repositories>`__ or elsewhere on GitHub.
* **Documentation issue tracker**: `ROS documentation repository <https://github.com/ros2/ros2_documentation/issues>`__.
* **Catch-all issue tracker**: `top level ROS repository <https://github.com/ros2/ros2>`__.

Prerequisites
-------------

There are no prerequisites.

Steps
-----

1 Finding an issue to triage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We encourage contributors to look at incoming issues on ROS repositories and triage the problems that users are having:

* To find code issues for triage, review the issues list of a repository under the `ROS organization <https://github.com/orgs/ros2/repositories>`__, for example the issues list in the `ROS CLI repository <https://github.com/ros2/ros2cli/issues/>`__.
* To find documentation issues for triage, review the issues list in the `ROS documentation repository <https://github.com/ros2/ros2_documentation>`__.
* If you're not sure where to start, review the issues list in the `top level ROS repository <https://github.com/ros2/ros2>`__.

When you have identified an issue for triage, add a comment to the issue to say that you are looking into it.

2 Confirming and reproducing the issue
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When you have found an issue to triage, confirm that the issue is genuinely an issue (a bug or a request for enhancement) and has been created with the expected information.

For more information about the information the reporter of the issue is expected to provide, see :doc:`./Reporting-an-issue`.

#. If the issue ``Description`` is actually posing a question, close the issue and add a comment directing the user to the `Robotics Stack Exchange <https://robotics.stackexchange.com/>`__ to get help with their question.
#. If the issue looks like a genuine issue (bug or enhancement), but is not relevant to the repository it has been raised in, transfer the issue to the appropriate ROS repository (`see the GitHub documentation for guidance about how to transfer between repositories <https://docs.github.com/en/issues/tracking-your-work-with-issues/administering-issues/transferring-an-issue-to-another-repository>`__).
#. If the issue appears to be a valid but lacks sufficient details to replicate the environment (for example, the host operating system, platform, RMW implementation, ROS distro, and offending code) please request that author provide additional information.
#. If the issue is a bug, try to reproduce the bug using the provided steps — if you cannot reproduce the bug, add a comment to the reporter asking for clarification.

3 Assigning and labelling the issue
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

After you've confirmed the issue is a genuine issue, and reproduced it, assign and label the issue appropriately to allow work on the issue to begin by a developer.

#. Assign the issue to the maintainer for the ROS repository in question.
#. Label the issue as a ``bug`` or ``enhancement``, as appropriate.
#. If this issue is a request for a new feature, label the issue with ``help-wanted``.
#. If the reporter has not provided enough information to determine the cause of the problem, label the issue as ``more-information-needed`` and leave a comment for the reporter, including:

   * What you have tried while investigating the problem.
   * In what way the information supplied was insufficient.
   * Any suggestions you may have about what additional information is needed.

Related content
---------------

* :doc:`../Contributing`
* :doc:`Reporting-an-issue`

FAQs
----

TBC
