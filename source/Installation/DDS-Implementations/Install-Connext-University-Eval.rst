RTI Connext DDS
===============

.. contents:: Table of Contents
   :depth: 1
   :local:

A full-suite installation of RTI Connext DDS is available for many additional platforms, for universities, evaluation, or purchase.
This installation includes diagnostic tools, layered services, and security.  See below for installation details.

RTI University Program
----------------------

University researchers and classroom users may be eligible for a free academic license through the `RTI University Program <https://www.rti.com/free-trial/university-program>`__.
This includes a one-year (renewable) license to the unabridged version of Connext DDS Secure, which includes diagnostic tools and layered services.
The university license application can be found `here <https://www.rti.com/free-trial/university-program>`__.


RTI Connext DDS Evaluation
--------------------------

To install RTI Connext DDS **version 6.0.1** Evalution:
 * Visit the `RTI Free Trial (6.0.1) site <https://www.rti.com/free-trial>`__.
 * Download the version(s) to match your environment.
 * Contact license@rti.com for an evaluation license.
 * Install RTI Connext 6.0.1 by running the installation program.  When finished, it will run the RTI Launcher.
 * Use the RTI Launcher to install the license file (rti_license.dat) if needed.  The launcher may also be used to launch the diagnostic tools and services.

Detailed instructions for each platform are provided below.

Ubuntu Linux source install
---------------------------

RTI Connext (version 6.0.1, amd64 only)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Deb packages provided in the ROS 2 apt repositories
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can install a deb package of RTI Connext available on the ROS 2 apt repositories.
You will need to accept a license from RTI.

.. code-block:: bash

   sudo apt update && sudo apt install -q -y ros-{DISTRO}-rmw-connextdds rti-connext-dds-6.0.1

Source the setup file to set the ``NDDSHOME`` environment variable.

.. code-block:: bash

   cd /opt/rti.com/rti_connext_dds-6.0.1/resource/scripts && source ./rtisetenv_x64Linux4gcc7.3.0.bash; cd -

Note: when using ``zsh`` you need to be in the directory of the script when sourcing it to have it work properly

Now you can build as normal and support for RTI will be built as well.

Official binary packages from RTI
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can install the Connext 6.0.1 package for Linux provided by RTI.

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

You can install a libraries-only deb package of RTI Connext 6.0.1, available from the OSRF Apt repository under a `non-commercial license <https://www.rti.com/ncl>`__.

To install the libs-only deb package:

.. code-block:: bash

   sudo apt update && sudo apt install -q -y ros-{DISTRO}-rmw-connextdds rti-connext-dds-6.0.1

You will need to accept a license agreement from RTI, and will find an 'rti_license.dat file in the installation.

Add the following line to your ``.bashrc`` file pointing to your copy of the license (and source it).

.. code-block:: bash

   export RTI_LICENSE_FILE=path/to/rti_license.dat

All options need you to source the setup file to set the ``NDDSHOME`` environment variable:

.. code-block:: bash

   cd /opt/rti.com/rti_connext_dds-6.0.1/resource/scripts && source ./rtisetenv_x64Linux4gcc7.3.0.bash; cd -

Note: the above may need modification to match your RTI installation location

If you want to install the Connext DDS-Security plugins please refer to :doc:`this page <./Install-Connext-Security-Plugins>`.

OSX source install
------------------

RTI Connext (6.0.1)
^^^^^^^^^^^^^^^^^^^

You also need a Java runtime installed to run the RTI code generator, which you can get `here <https://support.apple.com/kb/DL1572?locale=en_US>`__.

After installing, run RTI launcher and point it to your license file.

Source the setup file to set the ``NDDSHOME`` environment variable before building your workspace.

.. code-block:: bash

   source /Applications/rti_connext_dds-6.0.1/resource/scripts/rtisetenv_x64Darwin17clang9.0.bash

You may need to increase shared memory resources following https://community.rti.com/kb/osx510

If you want to install the Connext DDS-Security plugins please refer to :doc:`this page <./Install-Connext-Security-Plugins>`.

OSX binary install
------------------


Enable Connext support
^^^^^^^^^^^^^^^^^^^^^^

After installing, run RTI launcher and point it to your license file.

Source the setup file to set the ``NDDSHOME`` environment variable before building your workspace.

.. code-block:: bash

   source /Applications/rti_connext_dds-6.0.1/resource/scripts/rtisetenv_x64Darwin17clang9.0.bash

You may need to increase shared memory resources following https://community.rti.com/kb/osx510.

If you want to install the Connext DDS-Security plugins please refer to :doc:`this page <./Install-Connext-Security-Plugins>`.

Windows source install
----------------------

RTI Connext 6.0.1
^^^^^^^^^^^^^^^^^

After installing, use the RTI Launcher to load your license file.

Then before building ROS 2, set up the Connext environment:

.. code-block:: bash

   call "C:\Program Files\rti_connext_dds-6.0.1\resource\scripts\rtisetenv_x64Win64VS2017.bat"

Note that this path might need to be slightly altered depending on where you selected to install RTI Connext DDS, and which version of Visual Studio was selected.
The path above is the current default path as of version 6.0.1, but will change as the version numbers increment in the future.

If you want to install the Connext DDS-Security plugins please refer to :doc:`this page <./Install-Connext-Security-Plugins>`.

Windows binary install
----------------------


RTI Connext
^^^^^^^^^^^

After installing, run RTI launcher and point it to your license file.

Then before using ROS 2, set up the Connext environment:

.. code-block:: bash

   call "C:\Program Files\rti_connext_dds-6.0.1\resource\scripts\rtisetenv_x64Win64VS2017.bat"

If you want to install the Connext DDS-Security plugins please refer to :doc:`this page <./Install-Connext-Security-Plugins>`.
