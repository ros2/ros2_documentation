Making a PR
===========

TBC

**Area: ROS-framework, ROS-tools, ROS-capabilities | Content-type: how-to | Experience: beginner**

.. contents:: Table of Contents
   :depth: 2
   :local:

Summary
-------

Pull requests are proposals to merge changes into a project.
Making a pull request allows you to collaborate with other contributors, providing a space to discuss and review code changes before merging them. 


Prerequisites
-------------

#. Complete your code changes on a branch, in your fork of the `target ROS repository <https://github.com/ros2>`__.
#. :ref:`Run the tests <colcon-run-the-tests>` for your changes to ensure that the code complies with ROS style guidelines.

Steps
-----

1. Literally how to create a PR - CLI/Web/Desktop?

Pull request guidelines
-----------------------

Scope and focus
^^^^^^^^^^^^^^^

* Limit each pull request to a single, well defined change.
* Submit unrelated changes as separate pull requests.
* Keep patches small and avoid unnecessary or incidental changes.

For more information, see `How to write the perfect pull request <https://github.com/blog/1943-how-to-write-the-perfect-pull-request>`__.

Draft pull requests
^^^^^^^^^^^^^^^^^^^

* Use draft pull requests to request early feedback while work is in progress.
* Do not expect draft pull requests to be formally reviewed or merged until they are marked ready.
* If you want early feedback from a specific person on a draft pull request, mention them (using @) in the pull request description or in a comment on the pull request.

Commit history and squashing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Before merging, squash changes into a minimum number of clear, semantic commits to preserve a readable project history.
* You can create new commits while a pull request is under review.
* Do not squash commits while a pull request is under review, as reviewers may not notice changes which can lead to confusion.
* Squashing early during development provides no benefit, as commits will be squashed before merging regardless.

Dependent pull requests
^^^^^^^^^^^^^^^^^^^^^^^

* If a pull request depends on another pull request, clearly reference the dependency.
* Ensure dependent pull requests are merged in the correct order.

Forked pull requests
^^^^^^^^^^^^^^^^^^^^

* When opening a pull request from a fork, select the `option that allows upstream contributors <https://github.com/blog/2247-improving-collaboration-with-forks>__` to support efficient collaboration.