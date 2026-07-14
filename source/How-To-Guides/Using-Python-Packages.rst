.. redirect-from::

  Guides/Using-Python-Packages
  Tutorials/Using-Python-Packages

.. _PythonPackages:

Using Python Packages with ROS 2
================================

**Goal:** Explain how to interoperate with other Python packages from the ROS 2 ecosystem.

.. contents:: Contents
    :depth: 2
    :local:

.. note::

    A cautionary note, if you intended to use pre-packaged binaries (either ``deb`` files, or the binary archive distributions), the Python interpreter must match what was used to build the original binaries.
    If you intend to use something like ``virtualenv`` or ``pipenv``, make sure to use the system interpreter.
    If you use something like ``conda``, it is very likely that the interpreter will not match the system interpreter and will be incompatible with ROS 2 binaries.

Installing via ``rosdep``
-------------------------

The fastest way to include third-party Python packages is to use their corresponding rosdep keys, if available.
``rosdep`` keys can be checked via:

* https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml
* https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml

These ``rosdep`` keys can be added to your ``package.xml`` file, which indicates to the build system that your package (and dependent packages) depend on those keys.
In a new workspace, you can also quickly install all rosdep keys with:

.. code-block:: console

    $ rosdep install -yr --from-paths ./path/to/your/workspace

If there aren't currently ``rosdep`` keys for the package that you are interested in, it is possible to add them by following the `rosdep key contribution guide`_.

To learn more about the ``rosdep`` tool and how it works, consult the `rosdep documentation`_.

Installing via a package manager
--------------------------------

If you don't want to make a rosdep key, but the package is available in your system package manager (eg ``apt``), you can install and use the package that way:

.. code-block:: console

    $ sudo apt install python3-serial

If the package is available on `The Python Package Index (PyPI) <https://pypi.org/>`_ and you want to install globally on your system:

.. code-block:: console

    $ python3 -m pip install -U pyserial

If the package is available on PyPI and you want to install locally to your user:

.. code-block:: console

    $ python3 -m pip install -U --user pyserial


Installing via a virtual environment
------------------------------------

ROS 2 packages using the ``ament_python`` build type can be modified to install Python entry point scripts using a virtual environment interpreter.
This allows Python nodes to run with dependencies installed in virtual environments.

First, create a Colcon workspace:

.. code-block:: console

    $ mkdir -p ~/colcon_venv/src
    $ cd ~/colcon_venv/

There are several Python packages required to run your Python node using a virtual environment.
These packages are installed in your system environment.
Export these packages and their version dependencies to a ``requirements.txt`` file:

.. code-block:: console

    $ source /opt/ros/{DISTRO}/setup.bash # Source installation to ensure correct Python interpreter is used
    $ python3 -m pip freeze | grep -E '^(PyYAML|setuptools|typing_extensions)==' > requirements.txt


After exporting these requirements, setup your virtual environment:

.. code-block:: console

    $ virtualenv -p python3 ./venv # Make a virtual env and activate it
    $ source ./venv/bin/activate
    $ touch ./venv/COLCON_IGNORE # Make sure that colcon does not try to build the venv


Next, install the required Python packages:

.. code-block:: console

    $ python3 -m pip install -r requirements.txt

After, install the Python packages that you want in your virtual environment:

.. code-block:: console

    $ python3 -m pip install gtsam pyserial # ... etc.

Place the ROS 2 package containing your Python node into the ``src`` directory of your Colcon workspace.
Open ``setup.py`` and add the ``options`` field to the ``setup`` call:

.. code-block:: python

    options={
        'build_scripts': {
            'executable': '<YOUR_PYTHON_EXECUTABLE_PATH>'
        }
    }

Replace ``<YOUR_PYTHON_EXECUTABLE_PATH>`` with the path to your virtual environment Python interpreter.
If you do not know the path, you can find it by running ``which python`` in the terminal while the virtual environment is active.

.. note::

    All Python nodes in the modified ROS 2 package will run using the virtual environment Python interpreter.
    If you need your Python nodes to run using different Python packages, you should move those nodes to separate ROS 2 packages and create separate virtual environments.

Now you can build the ROS 2 package in your workspace and run your Python node that depends on packages installed in your virtual environment.

.. code-block:: console

    $ source /opt/ros/{DISTRO}/setup.bash # Source {DISTRO_TITLE} and build
    $ colcon build

Installing via a virtual environment with symbolic links
--------------------------------------------------------

If you want to run your Python nodes using dependencies installed in a virtual environment, but you also want to build your ROS 2 package using the ``--symlink-install`` option to create symbolic links to those Python nodes, then the process varies slightly.

First, create a Colcon workspace:

.. code-block:: console

    $ mkdir -p ~/colcon_venv/src
    $ cd ~/colcon_venv/

There are several required Python packages installed in your system environment Python.
Export these packages and their version dependencies to a ``requirements.txt`` file:

.. code-block:: console

    $ source /opt/ros/{DISTRO}/setup.bash # Source installation to ensure correct Python interpreter is used
    $ python3 -m pip freeze | grep -E '^(colcon-common-extensions|PyYAML|setuptools|typing_extensions)==' > requirements.txt

After exporting these requirements, setup your virtual environment:

.. code-block:: console

    $ virtualenv -p python3 ./venv # Make a virtual env and activate it
    $ source ./venv/bin/activate
    $ touch ./venv/COLCON_IGNORE # Make sure that colcon does not try to build the venv

Next, install the required Python packages:

.. code-block:: console

    $ python3 -m pip install -r requirements.txt

After, install the Python packages that you want in your virtual environment:

.. code-block:: console

    $ python3 -m pip install gtsam pyserial # ... etc.

Now, while your virtual environment is active, you can build your ROS 2 package and run your Python node that depends on packages installed in your virtual environment.
Replace ``<YOUR_PACKAGE>`` with the name of the package containing your Python node.

.. code-block:: console

    $ source /opt/ros/{DISTRO}/setup.bash # Source {DISTRO_TITLE} and build
    $ python3 -m colcon build --symlink-install --packages-select <YOUR_PACKAGE>

.. note::

    If you want to release your package using Bloom, you should add the packages you require to ``rosdep``, see the `rosdep key contribution guide`_.

.. _rosdep key contribution guide: http://docs.ros.org/en/independent/api/rosdep/html/contributing_rules.html

.. _rosdep documentation: http://docs.ros.org/en/independent/api/rosdep/html/
