.. _making-a-pull-request:

Making a Pull Request
=====================

.. contents:: Table of Contents
   :depth: 1
   :local:

Before you start
----------------

* For large changes, open an issue or post on `ROS Discourse <https://discourse.ros.org>`__ first.
* Check there is no open PR for the same change.
* Read the :doc:`Developer Guide <Developer-Guide>`.

Setup
-----

1. Fork the repository and clone it:

   .. code-block:: bash

      git clone https://github.com/YOUR_USERNAME/REPO_NAME.git
      cd REPO_NAME

2. Add upstream:

   .. code-block:: bash

      git remote add upstream https://github.com/ros2/REPO_NAME.git

3. Create a branch:

   .. code-block:: bash

      git checkout -b my-fix-branch

Making changes
--------------

* One fix or feature per PR.
* Follow :doc:`Code Style and Language Versions <Code-Style-Language-Versions>`.
* Add or update tests.
* Update docs if user-facing behavior changes.

Submitting
----------

1. Commit your changes:

   .. code-block:: bash

      git commit -m "fix: description of change"

2. Push to your fork:

   .. code-block:: bash

      git push origin my-fix-branch

3. Open a PR against the ``rolling`` branch.
4. Fill in the PR template.
5. Reference related issues with ``Closes #ISSUE_NUMBER``.

After submitting
----------------

* Address review comments promptly.
* Do not force-push after a review has started unless asked.
