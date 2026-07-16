Reviewing a pull request (PR) — how-to
======================================

All incoming code and documentation to ROS projects must be reviewed in a pull request.
This article explains how to prepare for and review a pull request submitted by a contributor.
After reading this article, you'll be able to ensure changes in a pull request meet the required standards.

**Area: contributing, community | Content-type: how-to | Experience: beginner, intermediate, expert**

.. contents:: Table of Contents
   :depth: 2
   :local:

Summary
-------

Reviewing a pull request (PR) from a contributor allows you to check that their changes meet the appropriate guidelines and standards.
Anyone is welcome to review and approve a pull request.
Changes are ready to merge after they have been approved.
Only a :doc:`Committer </The-ROS2-Project/Governance>` for the target repository can merge a pull request into that repository, and they will not do so until it has been approved.

Prerequisites
-------------

A code or documentation contributor has :doc:`made a pull request </The-ROS2-Project/Contributing/Contributing-to-code/Making-a-PR>` to merge their changes into one of `the ROS repositories <https://github.com/ros2>`__.

Steps
-----

1 Preparing for review
^^^^^^^^^^^^^^^^^^^^^^

* Anyone is welcome to review a pull request.

  A pull request generally requires two reviews before it can be merged.

* Treat reviewing a pull request as a collaborative activity involving the submitter and other developers, rather than a passive or one-way process.
* As a reviewer:

  * You can make small improvements to code or documentation in-place, such as fixing typos or addressing minor style issues.
  * You should make a best effort attempt to comment on the pull request within one week of submission.

* When you begin reviewing a pull request, leave a comment to let others know you are performing a review.

2 Reviewing the pull request
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#. Review the pull request against the following guidelines:

   * Confirm the code or documentation changes are appropriate for the repository.
   * Verify the code is correct and complete, and scoped to a single, well defined change.
   * Check that the pull request targets the default branch (usually ``rolling``).
   * If the changes are based on a design document, such as a `REP <https://reps.openrobotics.org/>`__, verify that the changes are consistent with the design.
   * For code changes, ensure that the changes:

     * Follow the :doc:`Developer guide <../Developer-Guide>`.
     * Follow the :doc:`Code style guide <../Code-Style-Language-Versions>`.
     * Include tests for the new feature or bug fix.

   * For documentation changes, ensure the changes follow the :doc:`documentation guidance </The-ROS2-Project/Contributing/Contributing-to-documentation>`.
   * Confirm that the Continuous Integration (CI) run for the pull request passes cleanly.

#. Provide your review comments.

   You can add review comments to the pull request for the submitter, or suggest changes directly in the pull request (`see the GitHub documentation for guidance <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/reviewing-changes-in-pull-requests/commenting-on-a-pull-request>`__).

#. Follow these guidelines to make sure your review comments are useful and actionable:

   * Start with high-level comments (for example, asking for refactoring or design changes), then move on to lower-level comments about specifics.
   * Consider providing the following types of comment:

     * **Positive feedback** — for example:

       ``Nice work on handling edge cases here — the early return makes the logic much easier to follow.``

     * **Questions** — for example:

       ``Just to make sure I'm not missing a requirement, is there a reason we're using a custom sorting function here instead of localeCompare?``

     * **Suggestions** — for example:

       ``You could simplify this loop using Array.map to make it more concise:``

       .. code-block:: javascript

         const names = users.map(user => user.name)

     * **Issues** — for example:

       ``This function doesn't handle the case where response is null, which could cause a runtime error — add a guard clause:``

       .. code-block:: javascript

         if (!response) {
           return [...];
         }

     * **Housekeeping** — a change that isn't related to the main purpose of the pull request, but helps to keep the repository healthy, for example:

       ``Since this file is already being updated, could we also remove the unused formatDate import at the top?``

     * **Minor details** — small, nitpicking details such as improving style or readability, for example:

       ``Minor naming suggestion; user_list could be named users to better reflect that it's a collection.``

   * Be clear about what you expect to happen in response to each comment, including whether the comment blocks merging the pull request, and whether you consider your request optional or required.
   * Remember to include positive feedback and thanks for the work done by the submitter, and always be constructive.

3 Approving and merging the pull request
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

After you have reviewed the pull request and provided feedback, the submitter may continue the discussion or iterate on their changes, adding new commits to the PR.

When you are satisfied with the changes and they are ready to be merged, approve the pull request (`see the GitHub documentation for guidance <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/reviewing-changes-in-pull-requests/approving-a-pull-request-with-required-reviews>`__).

* Anyone is welcome to review a pull request, even if it already has a review.
* A pull request must have at least one approval, and in most cases, two approvals, from a developer (other than the author) before it can be merged to the target branch.
* Only a Committer for the target repository can merge an approved pull request.

  * See the :doc:`current ROS Committers </The-ROS2-Project/Governance>` for the list of people with merge permissions for the target repository.

* If the pull request has any dependencies, ensure that dependent pull requests are merged in the correct order.

Related content
---------------

* :ref:`ROS development general principles <general-principles>`
* :doc:`Making-a-PR`
* `About pull request reviews <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/reviewing-changes-in-pull-requests/about-pull-request-reviews>`__
