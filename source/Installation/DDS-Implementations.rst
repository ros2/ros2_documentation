Installing DDS implementations
==============================

By default, ROS 2 uses DDS as its `middleware <https://design.ros2.org/articles/ros_on_dds.html>`__.
It is compatible with multiple DDS or RTPS (the DDS wire protocol) vendors.
There is currently support for eProsima's Fast RTPS, RTI's Connext DDS and Eclipse Cyclone DDS.
ADLINK's OpenSplice is no longer supported as of Foxy.
See https://ros.org/reps/rep-2000.html for supported DDS vendors by distribution.

For distros before Eloquent, the only bundled vendor is eProsima's Fast RTPS.
Since Eloquent, both Fast RTPS and Cyclone DDS are bundled, but Fast RTPS is still the default.
`Working with Eclipse Cyclone DDS <DDS-Implementations/Working-with-Eclipse-CycloneDDS>` explains how to utilize Cyclone DDS.

.. toctree::
   :hidden:
   :glob:

   DDS-Implementations/*

If you would like to use one of the other vendors you will need to install their software separately before building.
The ROS 2 build will automatically build support for vendors that have been installed and sourced correctly.

Once you've installed a new DDS vendor, you can change the vendor used at runtime: `Working with Multiple RMW Implementations </Tutorials/Working-with-multiple-RMW-implementations>`.

Detailed instructions for installing other DDS vendors are provided below.

.. contents:: Platforms / Installation types
   :depth: 1
   :local:

Linux source install
--------------------

RTI Connext (version 5.3.1, amd64 only)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Debian packages provided in the ROS 2 apt repositories
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can install a Debian package of RTI Connext available on the ROS 2 apt repositories.
You will need to accept a license from RTI.

.. code-block:: bash

   sudo apt install -q -y \
       rti-connext-dds-5.3.1  # from packages.ros.org/ros2/ubuntu

Source the setup file to set the ``NDDSHOME`` environment variable.

.. code-block:: bash

   cd /opt/rti.com/rti_connext_dds-5.3.1/resource/scripts && source ./rtisetenv_x64Linux3gcc5.4.0.bash; cd -

Note: when using ``zsh`` you need to be in the directory of the script when sourcing it to have it work properly

Now you can build as normal and support for RTI will be built as well.

Official binary packages from RTI
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can install the Connext 5.3.1 package for Linux provided by RTI, via options available for `university, purchase or evaluation <DDS-Implementations/Install-Connext-University-Eval>`

After downloading, use ``chmod +x`` on the ``.run`` executable and then execute it.
Note that if you're installing to a system directory use ``sudo`` as well.

The default location is ``~/rti_connext_dds-5.3.1``

After installation, run RTI launcher and point it to your license file (obtained from RTI).

Add the following line to your ``.bashrc`` file pointing to your copy of the license.

.. code-block:: bash

   export RTI_LICENSE_FILE=path/to/rti_license.dat

Source the setup file to set the ``NDDSHOME`` environment variable.

.. code-block:: bash

   source ~/rti_connext_dds-5.3.1/resource/scripts/rtisetenv_x64Linux3gcc5.4.0.bash

Now you can build as normal and support for RTI will be built as well.

Linux binary install
--------------------

RTI Connext (version 5.3.1, amd64 only)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To use RTI Connext DDS there are full-suite install options available for `university, purchase or evaluation <DDS-Implementations/Install-Connext-University-Eval>`
or you can install a libraries-only Debian package of RTI Connext 5.3.1, available from the OSRF Apt repository
under a `non-commercial license <https://www.rti.com/ncl>`__.

To install the libs-only Debian package:

.. code-block:: bash

   sudo apt update && sudo apt install -q -y rti-connext-dds-5.3.1

You will need to accept a license agreement from RTI, and will find an 'rti_license.dat file in the installation.

Add the following line to your ``.bashrc`` file pointing to your copy of the license (and source it).

.. code-block:: bash

   export RTI_LICENSE_FILE=path/to/rti_license.dat

All options need you to source the setup file to set the ``NDDSHOME`` environment variable:

.. code-block:: bash

   cd /opt/rti.com/rti_connext_dds-5.3.1/resource/scripts && source ./rtisetenv_x64Linux3gcc5.4.0.bash; cd -

Note: the above may need modification to match your RTI installation location

If you want to install the Connext DDS-Security plugins please refer to `this page <DDS-Implementations/Install-Connext-Security-Plugins>`.

OSX source install
------------------

RTI Connext (5.3)
^^^^^^^^^^^^^^^^^

If you would like to also build against RTI Connext DDS there are options available for `university, purchase or evaluation <DDS-Implementations/Install-Connext-University-Eval>`

You also need a Java runtime installed to run the RTI code generator, which you can get `here <https://support.apple.com/kb/DL1572?locale=en_US>`__.

After installing, run RTI launcher and point it to your license file.

Source the setup file to set the ``NDDSHOME`` environment variable before building your workspace.

The setup file and path will depend on your macOS version.

.. code-block:: bash

   # macOS 10.12 Sierra
   source /Applications/rti_connext_dds-5.3.1/resource/scripts/rtisetenv_x64Darwin16clang8.0.bash
   # macOS 10.13 High Sierra
   source /Applications/rti_connext_dds-5.3.1/resource/scripts/rtisetenv_x64Darwin17clang9.0.bash

You may need to increase shared memory resources following https://community.rti.com/kb/osx510

If you want to install the Connext DDS-Security plugins please refer to `this page <DDS-Implementations/Install-Connext-Security-Plugins>`.

OSX binary install
------------------


Enable Connext support
^^^^^^^^^^^^^^^^^^^^^^

To use RTI Connext DDS there are options available for `university, purchase or evaluation <DDS-Implementations/Install-Connext-University-Eval>`

After installing, run RTI launcher and point it to your license file.

Set the ``NDDSHOME`` environment variable:

.. code-block:: bash

   export NDDSHOME=/Applications/rti_connext_dds-5.3.1

You may need to increase shared memory resources following https://community.rti.com/kb/osx510.

If you want to install the Connext DDS-Security plugins please refer to `this page <DDS-Implementations/Install-Connext-Security-Plugins>`.

Windows source install
----------------------

RTI Connext 5.3
^^^^^^^^^^^^^^^

If you would like to also build against RTI Connext DDS there are options available for `university, purchase or evaluation <DDS-Implementations/Install-Connext-University-Eval>`

After installing, use the RTI Launcher to load your license file.

Then before building ROS 2, set up the Connext environment:

.. code-block:: bash

   call "C:\Program Files\rti_connext_dds-5.3.1\resource\scripts\rtisetenv_x64Win64VS2017.bat"

Note that this path might need to be slightly altered depending on where you selected to install RTI Connext DDS, and which version of Visual Studio was selected.
The path above is the current default path as of version 5.3.1, but will change as the version numbers increment in the future.

If you want to install the Connext DDS-Security plugins please refer to `this page <DDS-Implementations/Install-Connext-Security-Plugins>`.

Windows binary install
----------------------


RTI Connext
^^^^^^^^^^^

To use RTI Connext DDS there are options available for `university, purchase or evaluation <DDS-Implementations/Install-Connext-University-Eval>`

After installing, run RTI launcher and point it to your license file.

Set the ``NDDSHOME`` environment variable:

.. code-block:: bash

   set "NDDSHOME=C:\Program Files\rti_connext_dds-5.3.1"

If you want to install the Connext DDS-Security plugins please refer to `this page <DDS-Implementations/Install-Connext-Security-Plugins>`.
