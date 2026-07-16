Making a pull request (PR) — how-to
===================================

Pull requests are used to contribute code and documentation changes to ROS projects.
This article explains how to prepare and create a pull request from your fork of a ROS repository.
With this information, you'll be able to submit focused changes in a pull request, ready for review.

**Area: contributing, community | Content-type: how-to | Experience: beginner, intermediate, expert**

.. contents:: Table of Contents
   :depth: 2
   :local:

Summary
-------

`Pull requests (PRs) <https://docs.github.com/en/pull-requests>`__ are proposals to merge your changes into a ROS repository.
Making a pull request allows you to collaborate with other ROS contributors, providing a space to discuss and review your code changes before a ROS maintainer merges them.
Pull requests are welcome for any of `the ROS repositories <https://github.com/ros2>`__.

For more information about contribution etiquette, see :doc:`Contributing </The-ROS2-Project/Contributing>`.

Prerequisites
-------------

#. `Create a fork <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/fork-a-repo>`__ of the target ROS repository for your code changes.
#. Complete your code changes on a development branch taken from the **rolling** branch, in your fork of the `target ROS repository <https://github.com/ros2>`__.
#. Make sure your changes comply with ROS guidelines.

   * If your pull request is for a code change:

     * Make sure you've followed the guidance in the :doc:`Developer guide </The-ROS2-Project/Contributing/Developer-Guide>`.
     * Check that your code complies with the relevant section of the :doc:`Code style guide </The-ROS2-Project/Contributing/Code-Style-Language-Versions>`.
     * Make sure you've :ref:`run the tests <colcon-run-the-tests>` and the appropriate linter for your code changes.

   * If your pull request is for a documentation change:

     * Make sure you've followed the guidance in :doc:`/The-ROS2-Project/Contributing/Contributing-to-documentation`.

Steps
-----

1 Preparing the pull request
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use the following guidelines to prepare your pull request:

* **Scope and focus**
   * Limit each pull request to a single, well-defined change.
   * Submit unrelated changes as separate pull requests.
   * Keep patches small and avoid unnecessary or incidental changes.
* **Commit history and squashing**
   * Squash changes into a minimum number of clear, semantic commits to preserve a readable project history.
   * Don't squash commits while a pull request is under review, as reviewers may not notice changes which can lead to confusion.
   * You can create new commits while a pull request is under review.
* **Draft pull requests**
   * Use draft pull requests to request early feedback while work is in progress.
   * Don't expect draft pull requests to be formally reviewed or merged until you have marked them as ready.
   * If you want early feedback from a specific person on a draft pull request, mention them (using ``@``) in the pull request description or in a comment.
* **Mentions and references**
   * If your changes are based on a design document, such as a `REP <https://reps.openrobotics.org/>`__, mention other people involved in the design, such as those who reviewed the REP, in the pull request description.
   * If your pull request depends on another pull request, clearly reference the dependency in the pull request description.
     Ensure to mention the pull request ID using ``#`` notation.
   * If your changes are planned to be released with a specific version of ROS, include that version of ROS in the pull request description.
* **Documenting your code changes**
   * If your pull request is for code changes, try to make any relevant documentation updates (including API documentation, feature documentation, and release notes) in the same pull request.

2 Submitting the pull request
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#. Create a pull request from the branch containing your changes in your fork, to the **rolling** branch of the target ROS repository.
   You can create your pull request using the GitHub CLI, GitHub Desktop, or the GitHub web interface.

   For more information about creating a pull request from a fork, see `the GitHub documentation <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request-from-a-fork>`__.

   For more information about each of the available pull request methods, see `the GitHub documentation <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request>`__.

#. Populate the pull request by completing the sections shown in the description template, including:

   * **Description**: summarize your code changes, linking to related GitHub issue(s) and PRs by ID, highlighting any key points or areas of concern.
   * **Issue**: include the ID of the GitHub issue fixed by your changes, in the format ``Fixes #(issue)``.
     This ensures that the issue is automatically closed when the pull request is merged.
   * **Generative AI**: if this pull request was generated using Generative AI, specify the model and version (for example, GitHub Copilot v3.2).
   * **Additional information**: provide any context or details you think will be useful for understanding your changes.

#. Select the `Allow edits by maintainers <https://github.blog/news-insights/product-news/improving-collaboration-with-forks/>`__ checkbox, to help ROS maintainers make small changes directly when needed.

After you've submitted your pull request, other developers and contributors in the ROS community will review your changes, including checking against the relevant guidelines.

3 Responding to review comments
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When another developer or contributor adds a review comment or suggestion to your pull request, you will receive a notification from GitHub.

You can view and discuss review comments directly in GitHub (see `the GitHub documentation for assistance <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/reviewing-changes-in-pull-requests/viewing-a-pull-request-review>`__), and add further commits to your branch to address them when needed.
You can also directly accept any suggested changes in the pull request, which adds a new commit to your branch automatically (see `the GitHub documentation for how to accept suggested changes <https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/reviewing-changes-in-pull-requests/incorporating-feedback-in-your-pull-request>`__).

Discuss and iterate on your changes with this feedback, amending and updating your development branch with new commits as needed.
Aim to reply back to review comments within one week, so that you and the reviewers do not lose the context of your changes.

4 Merging the pull request
^^^^^^^^^^^^^^^^^^^^^^^^^^

After you've actioned any feedback, your pull request must be approved by a :doc:`Committer for the target ROS repository </The-ROS2-Project/Governance>` before it can be merged.

When the Committer approves your pull request, they will merge it to the target branch (usually **rolling**), and you will receive a notification from GitHub.

Your changes may also be backported to older distributions of ROS.

Related content
---------------

* :ref:`ROS development general principles <general-principles>`
* :doc:`Reviewing-a-PR`
