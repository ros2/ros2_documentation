.. _CreatingOrUpdatingDocs:

Creating or updating documentation
==================================

.. centered:: TBC

.. parsed-literal::

    Area: ROS-community | Content-type: how-to | Experience: beginner, intermediate, expert

.. contents:: Table of Contents
   :depth: 2
   :local:

Summary
-------

TBC

The following steps relate to updating the ROS 2 user documentation. 
If you are working on developing a ROS 2 package, then you must also create or update the documentation for that package.
   
For more information on how to document ROS 2 packages, see :doc:`/How-To-Guides/Documenting-a-ROS-2-Package`.

Planning documentation changes
------------------------------

When you see a change to the documentation you'd like to make, we recommend checking the `docs issue list <https://github.com/ros2/ros2_documentation/issues>`__ to see if your proposed update has already been tracked.
You can also check for issues relating to nearby updates you could make to the article at the same time.

If you are creating a new article, decide on the content type for the article before you start. 
Follow the structure of that content type in the relevant example topic from the :doc:`./Documentation-guidelines`.

For more information about the docs source, tools, and workflow to use when making your updates, see :doc:`./Documentation-tooling-and-workflow`.

Building the site locally
-------------------------

Set up the following prerequisites to build the docs site locally:

#. Create a `venv <https://docs.python.org/3/library/venv.html>`__ to build the documentation:

   .. code-block:: console

      $ python3 -m venv ros2doc  # create venv
      $ source ros2doc/bin/activate  # activate venv

#. Install requirements located in the ``requirements.txt`` file:

   .. tabs::

      .. group-tab:: Linux

         .. code-block:: console

            $ pip install -r requirements.txt -c constraints.txt

      .. group-tab:: macOS

         .. code-block:: console

            $ pip install -r requirements.txt -c constraints.txt

      .. group-tab:: Windows

         .. code-block:: console

            $ python -m pip install -r requirements.txt -c constraints.txt

#. In order for Sphinx to be able to generate diagrams, the ``dot`` command must be available:

   .. tabs::

      .. group-tab:: Linux

         .. code-block:: console

            $ sudo apt update ; sudo apt install graphviz

      .. group-tab:: macOS

         .. code-block:: console

            $ brew install graphviz

      .. group-tab:: Windows

         Download an installer from the `Graphviz Download page <https://graphviz.gitlab.io/_pages/Download/Download_windows.html>`__ and install it.
         Make sure to allow the installer to add it to the Windows ``%PATH%``, otherwise Sphinx will not be able to find it.

Building the site for one branch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To build the site for just this branch:

#. Run the following command at the top level of the repository.
   The build process can take some time.
   This is the recommended way to test out local changes.

   .. code-block:: console

      $ make html

#. Open ``build/html/index.html`` in your browser to see the output.


Checking / Testing the site
^^^^^^^^^^^^^^^^^^^^^^^^^^^

* You can run the documentation tests locally (using `doc8 <https://github.com/PyCQA/doc8>`_) with the following command:

  .. code-block:: console

     $ make test

* You can run the Python documentation tools tests locally (using `pytest <https://docs.pytest.org/en/stable/>`_) with the following command:

  .. code-block:: console

     $ make test-tools

* You can run the documentation linter locally (using `sphinx-lint <https://github.com/sphinx-contrib/sphinx-lint>`_) with the following command:

  .. code-block:: console

     $ make lint

* You can run the documentation spell checker locally (using `codespell <https://github.com/codespell-project/codespell>`_) with the following command:

  .. code-block:: console

     $ make spellcheck

.. note::

   If the spellcheck command detects a specific word that needs to be ignored, add it to `codespell_whitelist <https://github.com/ros2/ros2_documentation/blob/{REPOS_FILE_BRANCH}/codespell_whitelist.txt>`_ .

For more information about spelling checks, see :ref:`Spelling check <spelling-check>`.

View Site Through Github CI
^^^^^^^^^^^^^^^^^^^^^^^^^^^

For small changes to the ROS 2 Docs you can view your changes as rendered HTML using artifacts generated in our Github Actions.
The "build" action produces the entire ROS Docs as a downloadable zip file that contains all HTML for `docs.ros.org <https://docs.ros.org/>`_
This build action is triggered after passing the test action and lint action.

To download and view your changes:

#. Go to your pull request and under the title, click the "Checks" tab.
#. On the left hand side of the checks page, click the "Test" section.
#. Under the "tests" section, click "build" to open the build dialog.
#. In the menu on the right, click "Upload document artifacts".
#. Scroll to the bottom to see the download link for the zipped HTML files under heading "Artifact download URL".

.. image:: ./images/github_action.png
  :width: 100%
  :alt: Steps to find rendered HTML files on ROS Github action

Building the site for all branches
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To build the site for all branches:

Run the following command at the top level of the repository, from the ``rolling`` branch.

.. code-block:: console
   
   $ make multiversion

This has two drawbacks:

* The multiversion plugin doesn't understand how to do incremental builds, so it always rebuilds everything.
  This can be slow.

* The build process will always check out exactly the branches listed in the ``conf.py`` file.
  This means that local changes will not be shown.

To show local changes in the multiversion output:

#. Commit the changes to a local branch.
#. Edit the `conf.py <https://github.com/ros2/ros2_documentation/blob/rolling/conf.py>`_ file and change the ``smv_branch_whitelist`` variable to point to your branch.

Checking for broken links
^^^^^^^^^^^^^^^^^^^^^^^^^

To check for broken links on the site, run the following command:

.. code-block:: console

   $ make linkcheck

This will check the entire site for broken links, and output the results to the screen and ``build/linkcheck``.

.. _spelling-check:

Spelling check
^^^^^^^^^^^^^^

To scan the documentation files and flag any misspellings, run the following command:

.. code-block:: console

   $ make spellcheck

If errors are detected, review the suggestions and update the pull request as necessary.

Some words, such as technical terms or proper nouns, maybe mistakenly flagged as misspelled.
If you encounter such instances, you can add them to the ignore list to prevent them from being flagged in the future.
To do this, add the term or noun to the `codespell_whitelist <https://github.com/ros2/ros2_documentation/blob/{REPOS_FILE_BRANCH}/codespell_whitelist.txt>`_ file as follows:

.. code-block:: text

   empy
   jupyter
   lets
   ws

To include custom corrections that ``codespell`` should apply, you can add them to the `codespell_dictionary <https://github.com/ros2/ros2_documentation/blob/{REPOS_FILE_BRANCH}/codespell_dictionary.txt>`_ file as follows:

.. code-block:: text

   amnet->ament
   colcn->colcon
   rosabg->rosbag
   rosdistroy->rosdistro

To check the dictionaries, run the following command:

.. code-block:: console
   
   $ make check-dictionaries

This command checks the blank lines and leading/trailing spaces in the dictionaries.

If the check-dictionaries command complains about the dictionaries, run the following command:

.. code-block:: console
   
   $ make sort-dictionaries

This command automatically modifies the dictionaries if any issues are found.

Building the site with GitHub Codespaces
----------------------------------------

Before you can build the site with GitHub Codespaces, you need to have a GitHub account (if you don't have one, you can create one for free).

To build the site with GitHub Codespaces:

#. Go to the `ROS 2 Documentation GitHub repository <https://github.com/ros2/ros2_documentation>`__.
#. On the repository page, click "Code > Open with Codespaces" from the dropdown menu.
   You are redirected to your Codespaces page, where you can see the progress of the Codespaces creation.

   .. image:: images/codespaces.png
      :width: 100%
      :alt: Codespaces creation

When this completes, a Visual Studio Code tab is opened in your browser.
You can open the terminal by clicking on the "Terminal" tab in the top panel or by pressing :kbd:`Ctrl-J`.

In this terminal, you can run any command you want, for example, to build the site for just this branch:

.. code-block:: console

   $ make html

To view the site:

#. Click "Go Live" in the right bottom panel to open the site in a new tab in your browser.
#. Open ``build/html/index.html`` in your browser.

.. image:: images/live_server.png
   :width: 100%
   :alt: Live Server

Building the site with Devcontainer
-----------------------------------

The `ROS 2 Documentation GitHub repository <https://github.com/ros2/ros2_documentation>`__ also supports a ``Devcontainer`` development environment with Visual Studio Code.
This enables you to build the documentation without changing your operating system.

See :doc:`/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container` to install VS Code and Docker before the following procedure.

#. Clone repository and start VS Code:

   .. code-block:: console

      $ git clone https://github.com/ros2/ros2_documentation
      $ cd ./ros2_documentation
      $ code .

#. Install the "Remote Development" Extension within VS Code search in Extensions (CTRL+SHIFT+X).
#. Use ``View->Command Palette...`` or ``Ctrl+Shift+P`` to open the command palette.
#. Search for the command ``Dev Containers: Reopen in Container`` in the command palette and execute it.
   This builds your development docker container for you automatically.
#. Open a terminal using ``View->Terminal`` or ``Ctrl+Shift+``` and ``New Terminal`` in VS Code.
#. Inside the terminal, use the following command to build the documentation:

   .. code-block:: console

      $ make html

.. image:: images/vscode_devcontainer.png
   :width: 100%
   :alt: VS Code Devcontainer
