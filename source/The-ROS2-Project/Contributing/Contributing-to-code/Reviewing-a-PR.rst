Reviewing a pull request (PR) — how-to
======================================

All incoming code and documentation to ROS projects must be reviewed in a pull request.
This article explains how to prepare for and review a pull request submitted by a contributor.
After reading this article, you'll be able to ensure changes in a pull request meet the required standards.

**Area: community | Content-type: how-to | Experience: beginner, intermediate, expert**

.. contents:: Table of Contents
   :depth: 2
   :local:

Summary
-------

Reviewing a pull request (PR) from a contributor allows you to check that their changes meet the appropriate guidelines and standards.
Any developer is welcome to review and approve a pull request after review, when the changes are ready to merge.
Only a ROS core maintainer for the target repository can merge a pull request into that repository.

Prerequisites
-------------

A code or documentation contributor has made a pull request to merge their changes into one of `the ROS repositories <https://github.com/ros2>`__.

Steps
-----

1 Preparing for review
^^^^^^^^^^^^^^^^^^^^^^

* :ref:`Any developer <general-principles>` is welcome to review a pull request.

  A pull request generally requires two reviews before it can be merged.

* Treat reviewing a pull request as a collaborative activity involving the submitter and other developers, rather than a passive or one-way process.
* As a reviewer:

  * You can make small improvements to code or documentation in-place, such as fixing typos or addressing minor style issues.
  * You should make a best effort attempt to comment on the pull request within one week of submission.

* When you begin `reviewing a pull request <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/reviewing-changes-in-pull-requests/about-pull-request-reviews>`__, leave a comment to let others know it is under review.

2 Reviewing the pull request
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use the following guidelines to review the submitted pull request:

* Confirm the code or documentation changes are appropriate for the repository.
* Verify the code is correct and complete, and scoped to a single, well defined change.
* Check that the pull request targets the default branch (usually ``rolling``).
* If the changes are based on a design document, verify that the changes are consistent with the design.
* For code changes, ensure that the changes:

    * Follow the :doc:`Developer guide <../Developer-Guide>`.
    * Follow the :doc:`Code style guide <../Code-Style-Language-Versions>`.
    * Include tests for the new feature or bug fix.

* For documentation changes, ensure the changes follow the :doc:`documentation guidance <../Contributing-To-ROS-2-Documentation>`.
* Confirm that the Continuous Integration (CI) run for the pull request passes cleanly.

You can comment on the pull request for the submitter, or suggest changes directly in the pull request (`see the GitHub documentation for guidance <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/reviewing-changes-in-pull-requests/commenting-on-a-pull-request>`__).

3 Approving and merging the pull request
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

After you have reviewed the pull request and provided feedback, the submitter may continue the discussion or iterate on their changes, adding new commits to the PR.

When you are satisfied with the changes and they are ready to be merged, approve the pull request (`see the GitHub documentation for guidance <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/reviewing-changes-in-pull-requests/approving-a-pull-request-with-required-reviews>`__).

* :ref:`Any developer <general-principles>` is welcome to approve a pull request when the changes are ready to be merged.
* A pull request must have at least one approval from a developer (other than the author) before it can be merged to the target branch.
* Only a ROS core maintainer for the target repository can merge an approved pull request.

  * See the :doc:`current ROS PMC constituents and committers </The-ROS2-Project/Governance>` for the list of people with merge permissions for the target repository.

* If the pull request has any dependencies, ensure that dependent pull requests are merged in the correct order.
