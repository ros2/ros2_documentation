IDEs and Debugging [community-contributed]
==========================================

ROS 2 is not made around a specific development environment and the main focus is on building / running from the command line.
Nonetheless Integrated Development Environments (IDEs) can be used to develop, run and/or debug ROS 2 nodes.

Below are listed some IDEs and instructions on how to use them with ROS 2.


.. contents:: Contents
    :depth: 2
    :local:


General
-------


.. _InstalledPythonCode:

Installed Python Code
^^^^^^^^^^^^^^^^^^^^^

By default, when building workspaces with:

.. code-block:: console

   $ colcon build

The Python code will be coped over into the ``build``/``install`` directories.
So when attaching a debugger to a ``ros2 run`` command from within an IDE, the code being run (from the ``build``/``install``) is not the same as the files opened in the IDE project.

There are 2 options to deal with this:

* Open the source files from ``build``/``install`` directory and place breakpoints there.
* Build the workspace with the `--symlink-install <https://colcon.readthedocs.io/en/released/reference/verb/build.html#command-line-arguments>`__ flag to colcon, which will symlink the source files to the ``build``/``install`` directory instead.


Visual Studio Code
------------------

`VSCode <https://code.visualstudio.com/>`_ is a versatile and free development environment.

VSCode is relatively easy to use with ROS 2.
Simply activate your environment in a command line and start the VSCode application from the same terminal and use as normal.
So:

#. Create your ROS workspace as you would normally.
#. In a terminal, source both ROS 2 and your install (if it was built already).
#. Start VSCode from the same command line.
   The terminal will be blocked until the application is closed again.

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        $ source /opt/ros/{DISTRO}/setup.bash
        $ cd ~/dev_ws
        $ source ./install/setup.bash
        $ /usr/bin/code ./src/my_node/

   .. group-tab:: macOS

      .. code-block:: console

        $ . ~/ros2_install/ros2-osx/setup.bash
        $ cd ~/dev_ws
        $ . ./install/setup.bash
        $ /Applications/Visual Studio Code.app/Contents/Resources/app/bin/code ./src/my_node/

   .. group-tab:: Windows

      In a Windows command line interface:

      .. code-block:: console

        $ call C:\dev\ros2\local_setup.bat
        $ cd C:\dev_ws
        $ call .\install\local_setup.bat
        $ "C:\Program Files\Microsoft VS Code\Code.exe" .\src\my_node\

      Or in powershell:

      .. code-block:: console

        $ C:\dev\ros2\local_setup.ps1
        $ cd C:\dev_ws
        $ .\install\local_setup.ps1
        $ & "C:\Program Files\Microsoft VS Code\Code.exe" .\src\my_node\


VSCode and any terminal created inside VSCode will correctly inherit from the parent environment and should have ROS and installed package available.

.. note::

   After adding packages or making major changes you might need to source your install again.
   The simplest way to do this is to close VSCode and restart it as above.


Python
^^^^^^

In your workspace, verify the correct interpreter is used.
Through sourcing the basic command ``python`` should be correct, but VSCode likes to resort to an absolute path for Python.
In the bottom right corner click on "Selected Python Interpreter" to change it.

If your ROS 2 Python version is from a virtual environment, VSCode will try to source it at each run command.
But we already started VSCode from a sourced environment, so this extra step is not necessary.
You can disable this for the current workspace by finding "Settings" > "Extensions" > "Python" > "Activate Environment" and disabling the check.

Now simply run a file or create a configuration in ``launch.json``.
Debugging a node is easiest by creating a configuration like a ``python ...`` command, instead of ``ros2 run/launch ...``.
An example of ``launch.json`` could be:

.. code-block::

   {
       "version": "0.2.0",
       "configurations": [
           {
               "name": "Python: File",
               "type": "python",
               "request": "launch",
               "program": "my_node.py"
           },
       ]
   }


Instead you could also create a configuration for attaching to a running process, under "Attach using Process Id".


See :doc:`Setup ROS 2 with VSCode and Docker<Setup-ROS-2-with-VSCode-and-Docker-Container>` for full instructions on how to use VSCode, in combination with Docker.


PyCharm
-------

`PyCharm <https://www.jetbrains.com/pycharm/>`_ is an IDE specifically for Python.

Of course it can only be meaningfully used for nodes made in Python.

With PyCharm you can either attach to an existing process (probably started by you via ``ros2 run ...`` or ``ros2 launch ...``) or run the node directly from Python (equivalent to ``python [file.py]``.


Integrate for code inspection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can setup your PyCharm project such that it is fully aware of ROS 2 code, allowing code completion and suggestion.


Linux
"""""

Open a terminal, source ROS and start PyCharm:

.. code-block:: console

   $ source /opt/ros/humble/setup.bash
   $ cd path/to/dev_ws
   $ /opt/pycharm/bin/pycharm.sh

After selecting the correct interpreter, everything should work.

.. note::

    This is untested.


Windows
"""""""

First sourcing ROS and then starting PyCharm from the command line seems to have no effect on Windows.
Instead, some settings need to be tweaked.

#. Create your ROS workspace as you would normally.
#. Start PyCharm normally.
#. Open a project.
   This should be the root directory of the ROS node you're developing, e.g. ``C:\dev_ws\src\my_node``.
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


If there are dependencies built alongside with your package, they are probably not yet recognized and result in invalid IDE warnings and runtime errors.

Resolve this by:

* Making sure the ``PATH`` override in the run/debug configuration includes both the ROS 2 install and your workspace, e.g.:

  .. code-block:: console

     $ C:\dev\ros2_humble\local_setup.ps1
     $ C:\dev_ws\install\local_setup.ps1
     $ echo $ENV:Path

* Adding the relevant folders from the ``install/`` directory to your project sources.

  Go to "Settings..." and under "Project: " > "Project Structure" click "Add content root".
  Add all the relevant ``site-packages`` folders under ``install/Lib/*``.

  Finally, make sure your run/debug configuration has the option "include content roots in PYTHONPATH" enabled.

.. tip::

   Using the `--merge-install <https://colcon.readthedocs.io/en/released/user/isolated-vs-merged-workspaces.html>`__ option with your colcon build will limit the number of depending directories, making it easier to configure PyCharm.


Attach to Process
^^^^^^^^^^^^^^^^^

Even without any configuration to PyCharm, you can always just attach to a running Python node.
Open your project source and simply run your node as usual:

.. code-block:: console

   $ ros2 run my_node main

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
