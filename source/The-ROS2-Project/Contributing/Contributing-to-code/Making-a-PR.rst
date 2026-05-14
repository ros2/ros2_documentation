Making a PR
===========

Pull requests are used to contribute code changes to ROS projects.
This article explains how to prepare and create a pull request from your fork of a ROS repository.
With this information, you'll be able to submit focused changes in a pull request that are ready for review.

**Area: TBC | Content-type: how-to | Experience: beginner**

.. contents:: Table of Contents
   :depth: 2
   :local:

Summary
-------

Pull requests (PRs) are proposals to merge your changes into a ROS project.
Making a pull request allows you to collaborate with other ROS contributors, providing a space to discuss and review your ROS code changes before merging them.
Pull requests are welcome for any of `the ROS repositories <https://github.com/ros2>`__.

For more information about contribution etiquette, see :doc:`Contributing <The-ROS2-Project/Contributing>`.

Prerequisites
-------------

#. `Create a fork <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/fork-a-repo>`__ of the target ROS repository for your code changes.
#. Complete your code changes on a branch, in your fork of the `target ROS repository <https://github.com/ros2>`__.
#. :ref:`Run the tests <colcon-run-the-tests>` for your changes to ensure that the code complies with ROS style guidelines.

Steps
-----

1 Preparing the PR
^^^^^^^^^^^^^^^^^^

Use the following guidelines to prepare your pull request:

* **Scope and focus**
   * Limit each pull request to a single, well defined change.
   * Submit unrelated changes as separate pull requests.
   * Keep patches small and avoid unnecessary or incidental changes.
* **Commit history and squashing**
   * Before merging, squash changes into a minimum number of clear, semantic commits to preserve a readable project history.
   * Do not squash commits while a pull request is under review, as reviewers may not notice changes which can lead to confusion.
   * You can create new commits while a pull request is under review.
* **Draft pull requests**
   * Use draft pull requests to request early feedback while work is in progress.
   * Do not expect draft pull requests to be formally reviewed or merged until they are marked ready.
   * If you want early feedback from a specific person on a draft pull request, mention them (using @) in the pull request description or in a comment on the pull request.
* **Dependent pull requests**
   * If a pull request depends on another pull request, clearly reference the dependency in the pull request description.
   * Ensure dependent pull requests are merged in the correct order.
* **Documenting your changes**
   * Ensure you propose any required documentation updates (including API documentation, feature documentation, and release notes) for your code changes in another pull request.

If your pull request is for a code change, ensure you have followed the guidance in the :doc:`../Developer-Guide`.

If your pull request is for a documentation change, ensure you have followed the guidance in :doc:`../Contributing-To-ROS-2-Documentation`.

2 Submitting the PR
^^^^^^^^^^^^^^^^^^^

#. Create a pull request from the branch containing your changes in your fork, to the **rolling** branch of the target ROS repository.
   You can create your pull request using the GitHub CLI, GitHub Desktop, or using other methods.

   For more information about creating a pull request from a fork, see `the GitHub documentation <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request-from-a-fork>`__.

   For more information about each of the available pull request methods, see `the GitHub documentation <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request>`__.

#. Populate the pull request by completing the sections shown in the description template, including:

   * **Description**: summarise your code changes and the related GitHub issue, highlighting any key points or areas of concern.
   * **Issue**: include the ID of the GitHub issue fixed by your changes, in the format ``Fixes # (issue)``.
     This ensures that this issue is automatically closed when the pull request is merged.
   * **Generative AI**: if this pull request was generated using Generative AI, specify the model and version (for example, GitHub Copilot v3.2).
   * **Additional information**: provide any additional context or details about your changes which are needed.

After you have submitted your pull request, other developers and contributors in the ROS community will :doc:`review your changes <Reviewing-a-PR>`, including checking against the guidelines in this article.

3 Responding to review comments
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When another developer or contributor adds a review comment or suggestion to your pull request, you receive a notification from GitHub.
You can view and discuss `review comments directly in GitHub <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/reviewing-changes-in-pull-requests/viewing-a-pull-request-review>`__, and add further commits to your branch to address them when needed.
You can also directly `accept any suggested changes <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/reviewing-changes-in-pull-requests/incorporating-feedback-in-your-pull-request>`__ in the pull request, which adds a new commit to your branch automatically.

After you've actioned any review comments or suggested changes, your pull request must be approved by a core maintainer for the target ROS repository before it can be merged.
When the core maintainer has approved your pull request, the changes are merged to the target branch, and you receive another notification from GitHub.
