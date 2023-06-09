DDS implementations
===================

By default, ROS 2 uses DDS as its `middleware <https://design.ros2.org/articles/ros_on_dds.html>`__.
It is compatible with multiple DDS or RTPS (the DDS wire protocol) vendors.
There is currently support for eProsima's Fast DDS, RTI's Connext DDS, Eclipse Cyclone DDS, and GurumNetworks GurumDDS.
See https://ros.org/reps/rep-2000.html for supported DDS vendors by distribution.

The default DDS vendor is eProsima's Fast DDS.

* :doc:`Working with Eclipse Cyclone DDS <DDS-Implementations/Working-with-Eclipse-CycloneDDS>` explains how to utilize Cyclone DDS.
* :doc:`Working with eProsima Fast DDS <DDS-Implementations/Working-with-eProsima-Fast-DDS>` explains how to utilize Fast DDS.
* :doc:`Working with GurumNetworks GurumDDS <DDS-Implementations/Working-with-GurumNetworks-GurumDDS>` explains how to utilize GurumDDS.

.. toctree::
   :hidden:
   :glob:

   DDS-Implementations/*

If you would like to use one of the other vendors you will need to install their software separately before building.
The ROS 2 build will automatically build support for vendors that have been installed and sourced correctly.

Once you've installed a new DDS vendor, you can change the vendor used at runtime: :doc:`Working with Multiple RMW Implementations <../How-To-Guides/Working-with-multiple-RMW-implementations>`.

Detailed instructions for installing other DDS vendors are provided below.

.. contents:: Platforms / Installation types
   :depth: 1
   :local:

Ubuntu Linux source install
---------------------------

RTI Connext (version 6.0.1, amd64 only)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Debian packages provided in the ROS 2 apt repositories
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can install a Debian package of RTI Connext available on the ROS 2 apt repositories.
You will need to accept a license from RTI.

.. code-block:: bash

   sudo apt update && sudo apt install -q -y rti-connext-dds-6.0.1

Source the setup file to set the ``NDDSHOME`` environment variable.

.. code-block:: bash

   cd /opt/rti.com/rti_connext_dds-6.0.1/resource/scripts && source ./rtisetenv_x64Linux4gcc7.3.0.bash; cd -

Note: when using ``zsh`` you need to be in the directory of the script when sourcing it to have it work properly

Now you can build as normal and support for RTI will be built as well.

Official binary packages from RTI
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can install the Connext 6.0.1 package for Linux provided by RTI, via options available for :doc:`university, purchase or evaluation <DDS-Implementations/Install-Connext-University-Eval>`

After downloading, use ``chmod +x`` on the ``.run`` executable and then execute it.
Note that if you're installing to a system directory use ``sudo`` as well.

The default location is ``~/rti_connext_dds-6.0.1``

After installation, run RTI launcher and point it to your license file (obtained from RTI).

Add the following line to your ``.bashrc`` file pointing to your copy of the license.

.. code-block:: bash

   export RTI_LICENSE_FILE=path/to/rti_license.dat

Source the setup file to set the ``NDDSHOME`` environment variable.

.. code-block:: bash

   cd ~/rti_connext_dds-6.0.1/resource/scripts && source ./rtisetenv_x64Linux4gcc7.3.0.bash; cd -

Now you can build as normal and support for RTI will be built as well.

Ubuntu Linux binary install
---------------------------

RTI Connext (version 6.0.1, amd64 only)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To use RTI Connext DDS there are full-suite install options available for :doc:`university, purchase or evaluation <DDS-Implementations/Install-Connext-University-Eval>`
or you can install a libraries-only Debian package of RTI Connext 6.0.1, available from the OSRF Apt repository
under a `non-commercial license <https://www.rti.com/ncl>`__.

To install the libs-only Debian package:

.. code-block:: bash

   sudo apt update && sudo apt install -q -y rti-connext-dds-6.0.1

You will need to accept a license agreement from RTI, and will find an 'rti_license.dat file in the installation.

Add the following line to your ``.bashrc`` file pointing to your copy of the license (and source it).

.. code-block:: bash

   export RTI_LICENSE_FILE=path/to/rti_license.dat

All options need you to source the setup file to set the ``NDDSHOME`` environment variable:

.. code-block:: bash

   cd /opt/rti.com/rti_connext_dds-6.0.1/resource/scripts && source ./rtisetenv_x64Linux4gcc7.3.0.bash; cd -

Note: the above may need modification to match your RTI installation location

If you want to install the Connext DDS-Security plugins please refer to :doc:`this page <DDS-Implementations/Install-Connext-Security-Plugins>`.

OSX source install
------------------

RTI Connext (6.0.1)
^^^^^^^^^^^^^^^^^^^

If you would like to also build against RTI Connext DDS there are options available for :doc:`university, purchase or evaluation <DDS-Implementations/Install-Connext-University-Eval>`

You also need a Java runtime installed to run the RTI code generator, which you can get `here <https://support.apple.com/kb/DL1572?locale=en_US>`__.

After installing, run RTI launcher and point it to your license file.

Source the setup file to set the ``NDDSHOME`` environment variable before building your workspace.

.. code-block:: bash

   source /Applications/rti_connext_dds-6.0.1/resource/scripts/rtisetenv_x64Darwin17clang9.0.bash

You may need to increase shared memory resources following https://community.rti.com/kb/osx510

If you want to install the Connext DDS-Security plugins please refer to :doc:`this page <DDS-Implementations/Install-Connext-Security-Plugins>`.

OSX binary install
------------------


Enable Connext support
^^^^^^^^^^^^^^^^^^^^^^

To use RTI Connext DDS there are options available for :doc:`university, purchase or evaluation <DDS-Implementations/Install-Connext-University-Eval>`

After installing, run RTI launcher and point it to your license file.

Source the setup file to set the ``NDDSHOME`` environment variable before building your workspace.

.. code-block:: bash

   source /Applications/rti_connext_dds-6.0.1/resource/scripts/rtisetenv_x64Darwin17clang9.0.bash

You may need to increase shared memory resources following https://community.rti.com/kb/osx510.

If you want to install the Connext DDS-Security plugins please refer to :doc:`this page <DDS-Implementations/Install-Connext-Security-Plugins>`.

Windows source install
----------------------

RTI Connext 6.0.1
^^^^^^^^^^^^^^^^^

If you would like to also build against RTI Connext DDS there are options available for :doc:`university, purchase or evaluation <DDS-Implementations/Install-Connext-University-Eval>`

After installing, use the RTI Launcher to load your license file.

Then before building ROS 2, set up the Connext environment:

.. code-block:: bash

   call "C:\Program Files\rti_connext_dds-6.0.1\resource\scripts\rtisetenv_x64Win64VS2017.bat"

Note that this path might need to be slightly altered depending on where you selected to install RTI Connext DDS, and which version of Visual Studio was selected.
The path above is the current default path as of version 6.0.1, but will change as the version numbers increment in the future.

If you want to install the Connext DDS-Security plugins please refer to :doc:`this page <DDS-Implementations/Install-Connext-Security-Plugins>`.

Windows binary install
----------------------


RTI Connext
^^^^^^^^^^^

To use RTI Connext DDS there are options available for :doc:`university, purchase or evaluation <DDS-Implementations/Install-Connext-University-Eval>`

After installing, run RTI launcher and point it to your license file.

Then before using ROS 2, set up the Connext environment:

.. code-block:: bash

   call "C:\Program Files\rti_connext_dds-6.0.1\resource\scripts\rtisetenv_x64Win64VS2017.bat"

If you want to install the Connext DDS-Security plugins please refer to :doc:`this page <DDS-Implementations/Install-Connext-Security-Plugins>`.
