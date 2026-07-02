.. _reviewing-a-pull-request:

Reviewing a Pull Request
========================

.. contents:: Table of Contents
   :depth: 1
   :local:

Anyone can review a PR.
You do not need to be a maintainer.

Finding PRs to review
---------------------

* Browse open PRs in any `ros2 <https://github.com/ros2>`__ or `ament <https://github.com/ament>`__ repository.
* Look for PRs labeled ``needs-review``.

What to check
-------------

* Does the change do what the description says?
* Are edge cases handled?
* Do tests cover the new behavior?
* Does the code follow the :doc:`style guidelines <Code-Style-Language-Versions>`?
* Is documentation updated if behavior changed?

How to review
-------------

1. Open the PR on GitHub.
2. Click **Files changed**.
3. Click the ``+`` icon on a line to leave a comment.
4. Click **Review changes** when done.
5. Choose **Comment**, **Approve**, or **Request changes**.

Review etiquette
----------------

* Be specific about what needs to change and why.
* Separate blocking issues from minor suggestions.
* Do not nitpick things that automated tools already catch.
