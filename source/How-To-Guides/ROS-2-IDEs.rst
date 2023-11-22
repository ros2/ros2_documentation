IDEs and Debugging
==================

ROS 2 is not made around a specific development environment and the main focus is on building / running from the command line.
Nonetheless Integrated Development Environments (IDEs) can be used to develop, run and/or debug ROS 2 nodes.

Below are listed some IDEs and instructions on how to use them with ROS 2, if at all possible.


.. contents:: Contents
    :depth: 2
    :local:


General
-------


.. _InstalledPythonCode:

Installed Python Code
^^^^^^^^^^^^^^^^^^^^^

By default, when building workspaces with:

.. code-block:: bash

   colcon build

The Python code will be coped over into the ``build``/``install`` directories.
So when attaching a debugger to a ``ros2 run`` command from within an IDE, the code being run (from the ``build``/``install``) is not the same as the files opened in the IDE project.

There are 2 options to deal with this:

* Open the source files from ``build``/``install`` directory and place breakpoints there.
* Build the workspace with the `--symlink-install <https://colcon.readthedocs.io/en/released/reference/verb/build.html#command-line-arguments>`__ flag to colcon, which will symlink the source files to the ``build``/``install`` directory instead.


Visual Studio Code
------------------

`VSCode <https://code.visualstudio.com/>`_ is a versatile and free development environment.

See :doc:`Setup ROS2 with VSCode and Docker<Setup-ROS-2-with-VSCode-and-Docker-Container>` for full instructions on how to use VSCode, in combination with Docker.


PyCharm
-------

`PyCharm <https://www.jetbrains.com/pycharm/>`_ is an IDE specifically for Python.

Of course it can only be meaningfully used for nodes made in Python.

.. note::

   With PyCharm you can either attach to an existing process (probably started by you via ``ros2 run ...`` or ``ros2 launch ...``) or run the node directly from Python (equivalent to ``python [file.py]``.


Integrate for code inspection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can setup your PyCharm project such that it is fully aware of ROS 2 code, allowing code completion and suggestion.


Linux
"""""

Open a terminal, source ROS and start PyCharm:

.. code-block:: bash

   source /opt/ros/humble/setup.bash
   cd ~/dev_ws
   /opt/pycharm/bin/pycharm.sh

After selecting the correct interpreter, everything should work.

.. note::

    This is untested.


Windows
"""""""

First sourcing ROS and then starting PyCharm from the command line seems to have no effect on Windows.
Instead, some settings need to be tweaked.

#. Create your ROS workspace as you would normally.
#. Start PyCharm normally.
#. Open a project. This should be the root directory of the ROS node you're developping, e.g. ``~/dev_ws/src/my_node``.
#. Click "Add new interpreter" > "Add local interpreter...".
   Select a system interpreter (or virtual environment if you're using one) and select the executable of your ROS Python version (typically ``C:\Python38\python.exe``).

      * If you now open one of your code files, you will see warnings about missing imports.
        Trying to run the file will confirm these issues.

#. Under the "Python Interpreters" window, find and select your ROS interpreter.
   Edit the name to something recognizable.
   More importantly, now click the "Show Interpreter Paths" button.
#. In the new window, you will see the paths already associated with this interpreter.
   Click the "+" button and add two more paths (according to your ROS install):

      * ``C:\dev\ros2_humble\bin``
      * ``C:\dev\ros2_humble\Lib\site-packages``

PyCharm will re-index and when finished it should correctly interpret your project, recognising the ROS 2 system packages.
You can navigate through code, get completion and read doc blurbs as expected.


Attach to Process
^^^^^^^^^^^^^^^^^

Even without any configuration to PyCharm, you can always just attach to a running Python node.
Open your project source and simply run your node as usual:

.. code-block:: bash

   ros2 run my_node main

Then in PyCharm select "Run" > "Attach to Process...".
It might take a second, but a small window should show listing the currently running Python instances, including your node.
There can be multiple Python processes, so there may be some trial-and-error to find the right one.

After selecting an instance, the usual debugging tools are available.
You can pause it or create breakpoints in the code and step through it.

.. note::

   The code in your project might not be the files being executed, see :ref:`this<InstalledPythonCode>`.


Run/Debug
^^^^^^^^^

Follow the steps for integration first.

Running your Python file from PyCharm will likely result in import errors.
This is because PyCharm extends the ``PYTHONPATH`` environment variable, but it leaves ``PATH`` untouched.
Necessary library files in ``ros/bin`` are not found.

Edit the run/debug configuration for your file and under "Environment Variables:" add a new variable.
It is currently not supported to extend the existing ``PATH``, so we need to override it.
From a sourced ROS terminal, export the content of ``PATH`` with: ``echo $Env:PATH``.
Copy the result.

Back in PyCharm, paste it as ``PATH``, apply changes and run or debug your node.
It should work like any Python project now, allowing easy additions of breakpoints and other debug methods.

.. note::

   On Windows it seems the capitalization of the ``PATH`` variable under "Environment Variables:" must be "path" (all lowercase) in order to work.
