Making a PR
===========

Pull requests are used to contribute code changes to ROS projects.
This article explains how to prepare and create a pull request from your fork of a ROS repository.
With this information, you'll be able to submit focused changes that are ready for review.

**Area: TBC | Content-type: how-to | Experience: beginner**

.. contents:: Table of Contents
   :depth: 2
   :local:

Summary
-------

Pull requests are proposals to merge your changes into a ROS project.
Making a pull request allows you to collaborate with other ROS contributors, providing a space to discuss and review code changes before merging them.


Prerequisites
-------------

#. `Create a fork <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/fork-a-repo>`__ of the target ROS repository for your code changes.
#. Complete your code changes on a branch, in your fork of the `target ROS repository <https://github.com/ros2>`__.
#. :ref:`Run the tests <colcon-run-the-tests>` for your changes to ensure that the code complies with ROS style guidelines.

Steps
-----

#. Create a pull request from the branch containing your changes in your fork, to the **rolling** branch of the target repository.

   For more information about creating a pull request from a fork, see `the GitHub documentation <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request-from-a-fork>`__.

#. You can create your pull request using the GitHub CLI, GitHub Desktop, or using other methods.

   For more information about each of these methods, see `the GitHub documentation <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request>`__.

Pull request guidelines
-----------------------

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