.. redirect-from::

    Source-Control-Best-Practices
    Contributing/Source-Control-Best-Practices

Source Control Best Practices
=============================

.. contents:: Table of Contents
   :depth: 2
   :local:

This page highlights source control considerations commonly
encountered when contributing to ROS 2 projects.
It is not intended to replace Git documentation.
Instead, it focuses on ROS-specific recommendations and
references to external resources where appropriate.

Avoid committing generated workspace artifacts
----------------------------------------------

ROS 2 workspaces generate build artifacts that should
generally not be committed to source control.

Common examples include:

.. code-block:: text

   build/
   install/
   log/

These directories are commonly generated when
building ROS 2 workspaces with ``colcon build``.
They can be recreated locally and
generally should not be committed to source control.
Repository ``.gitignore`` files should exclude them.

Before committing changes, review the files included in a commit
to ensure that generated workspace artifacts have not been added accidentally.

Editor-specific files
---------------------

Editors and IDEs often generate project-specific or
user-specific configuration files.

Common examples include:

.. code-block:: text

   .vscode/
   .idea/

In most ROS 2 repositories, editor-specific files are
not committed to source control.

Many ROS 2 repositories ignore editor-specific
files such as ``.vscode/`` and ``.idea/``.
Contributors should avoid committing personal editor
configuration unless it is explicitly required by the repository.

Using a global gitignore
------------------------

A global gitignore can help prevent accidentally committing
files that are unrelated to a repository, such as
operating system metadata files or personal editor configuration.

Git supports configuring a global excludes file
through the ``core.excludesfile`` configuration option.

For example:

.. code-block:: console

   $ git config --global core.excludesfile ~/.gitignore_global

A global gitignore is useful for excluding files that are
specific to a developer's machine or editor and
are not intended to be committed to any repository.

Authentication and credentials
------------------------------

When contributing to ROS 2 repositories,
contributors will often interact with Git
hosting services such as GitHub.

SSH keys and Git credential helpers can simplify authentication.
Avoid repeatedly entering credentials when
pushing changes or interacting with remote repositories.

Rather than duplicating setup instructions here,
refer to the official GitHub and Git documentation
for recommended configuration steps.

Additional resources
--------------------

* `Git ignore documentation <https://git-scm.com/docs/gitignore>`_
* `GitHub gitignore templates <https://github.com/github/gitignore>`_
* `GitHub SSH documentation <https://docs.github.com/en/authentication/connecting-to-github-with-ssh>`_
* `Git credential storage documentation <https://git-scm.com/book/en/v2/Git-Tools-Credential-Storage>`_
