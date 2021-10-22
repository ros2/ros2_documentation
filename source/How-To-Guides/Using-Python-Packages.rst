.. redirect-from::

  Guides/Using-Python-Packages
  Tutorials/Using-Python-Packages

.. _PythonPackages:

Using Python Packages with ROS 2
================================

**Goal:** Explain how to interoperate with other Python packages from the ROS2 ecosystem.

.. contents:: Contents
    :depth: 2
    :local:

.. note::

    A cautionary note, if you intended to use pre-packaged binaries (either ``deb`` files, or the “fat” binary distributions), the Python interpreter must match what was used to build the original binaries. If you intend to use something like ``virtualenv`` or ``pipenv``\, make sure to use the system interpreter.  If you use something like ``conda``, it is very likely that the interpreter will not match the system interpreter and will be incompatible with ROS2 binaries.

Installing via ``rosdep``
-------------------------

The fastest way to include third-party python packages is to use their corresponding rosdep keys, if available.  ``rosdep`` keys can be checked via:

* https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml
* https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml

These ``rosdep`` keys can be added to your ``package.xml`` file, which indicates to the build system that your package (and dependent packages) depend on those keys. In a new workspace, you can also quickly install all rosdep keys with:

.. code-block:: console

    rosdep install -yr ./path/to/your/workspace

If there aren’t currently ``rosdep`` keys for the package that you are interested in, it is possible to add them by following the `rosdep key contribution guide`_.

To learn more about the ``rosdep`` tool and how it works, consult the `rosdep documentation`_.

Installing via a package manager
--------------------------------

If you don’t want to make a rosdep key, but the package is available in your system package manager (eg ``apt``), you can install and use the package that way:

.. code-block:: console

    sudo apt install python3-serial

If the package is available on `The Python Package Index (PyPi) <https://pypi.org/>`_ and you want to install globally on your system:

.. code-block::

    python3 -m pip install -U pyserial

If the package is available on PyPi and you want to install locally to your user:

.. code-block:: console

    python3 -m pip install -U --user pyserial

Installing via a virtual environment
------------------------------------

First, create a Colcon workspace:

.. code-block:: console

    mkdir -p ~/colcon_venv/src
    cd ~/colcon_venv/

Then setup your virtual environment:

.. code-block:: console

    # Make a virtual env and activate it
    virtualenv -p python3 ./venv
    source ./venv/bin/activate
    # Make sure that colcon doesn’t try to build the venv
    touch ./venv/COLCON_IGNORE

Next, install the Python packages that you want in your virtual environment:

.. code-block:: console

    python3 -m pip install gtsam pyserial… etc

Now you can build your workspace and run your python node that depends on packages installed in your virtual environment.

.. code-block:: console

    # Source Foxy and build
    source /opt/ros/foxy/setup.bash
    colcon build

.. note::

    If you want release your package on Bloom, you should to add the packages you require to ``rosdep``, see the `rosdep key contribution guide`_.

.. _rosdep key contribution guide: http://docs.ros.org/en/independent/api/rosdep/html/contributing_rules.html

.. _rosdep documentation: http://docs.ros.org/en/independent/api/rosdep/html/
