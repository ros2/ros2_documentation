.. redirect-from::

    Building-ROS-2-on-Linux-with-Eclipse-Oxygen

Building ROS 2 on Linux with Eclipse Oxygen [community-contributed]
===================================================================

.. warning::
   **Some people have reported issues with this tutorial.
   If the steps work for you please leave a comment on https://github.com/ros2/ros2/issues/495 .
   If the steps don't work then please comment with the first step that didn't work.**

This tutorial is based on a clean ubuntu-16.04.2 install and eclipse oxygen with egit. It uses RTI Connext as middleware for Realtime performance. The `original Install page <../Installation/Ubuntu-Development-Setup>` is perhaps more up-to-date, so check it for info.

Install:

.. code-block:: bash

   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/dashing.list'
   sudo apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

.. code-block:: bash

   sudo apt update
   sudo apt install git wget build-essential cppcheck cmake libopencv-dev python-empy python3-dev python3-empy python3-nose python3-pip python3-pyparsing python3-setuptools python3-vcstool python3-yaml libtinyxml-dev libeigen3-dev clang-format pydocstyle pyflakes python3-coverage python3-mock python3-pep8 uncrustify libasio-dev libtinyxml2-dev libcurl4-openssl-dev libqt5core5a libqt5gui5 libqt5opengl5 libqt5widgets5 libxaw7-dev libgles2-mesa-dev libglu1-mesa-dev qtbase5-dev

Then install

.. code-block:: bash

   sudo pip3 install argcomplete flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes pytest pytest-cov pytest-runner

Create an eclipse workspace named ros2_ws (the name does not need to be ros2_ws).

.. image:: https://i.imgur.com/sdN8cab.png
   :target: https://i.imgur.com/sdN8cab.png
   :alt: eclipse-1


Inside this eclipse-workspace create a C++ Project.
This option creates an indexer for code-completion.

.. image:: https://i.imgur.com/TDsxpVS.png
   :target: https://i.imgur.com/TDsxpVS.png
   :alt: eclipse-1


Name the project ros2_ws.

.. image:: https://i.imgur.com/4db7JQI.png
   :target: https://i.imgur.com/4db7JQI.png
   :alt: eclipse-1


The project and some includes are shown.
The includes don't reside in our workspace, so don't remove them, they are for the indexer.

.. image:: https://i.imgur.com/RsllCLW.png
   :target: https://i.imgur.com/RsllCLW.png
   :alt: eclipse-1


Create a folder inside our project, named "src".

.. image:: https://i.imgur.com/WUGDQvB.png
   :target: https://i.imgur.com/WUGDQvB.png
   :alt: eclipse-1


The folder now exists in the project.
This folder also exist in the workspace.
Next go to a console and switch to the directory /home/ros/ros2_ws/ros2_ws.
There enter:

.. code-block:: bash

   wget https://raw.githubusercontent.com/ros2/ros2/dashing/ros2.repos
   vcs-import src < ros2.repos

Add export RTI_LICENSE_FILE=/home/ros/rti_connext_dds-5.3.1/rti_license.dat to .bashrc and source it.


.. image:: https://i.imgur.com/AtT6pWi.png
   :target: https://i.imgur.com/AtT6pWi.png
   :alt: eclipse-1


An RTI license is now needed, which we can get on their website.
Refer to `Ubuntu Linux Development Setup page <../Installation/Ubuntu-Development-Setup>`.
The RTI license file will be directly sent via email after sign-up.

In the email is a link to the RTI software to download.
Download the file, then run "chmod +x" on it to make it runnable.

.. image:: https://i.imgur.com/daIBmJA.png
   :target: https://i.imgur.com/daIBmJA.png
   :alt: eclipse-1



.. image:: https://i.imgur.com/ji7Wfl6.png
   :target: https://i.imgur.com/ji7Wfl6.png
   :alt: eclipse-1


Choose the home directory to install.

.. image:: https://i.imgur.com/8pE0GAX.png
   :target: https://i.imgur.com/8pE0GAX.png
   :alt: eclipse-1



.. image:: https://i.imgur.com/tgIxhWz.png
   :target: https://i.imgur.com/tgIxhWz.png
   :alt: eclipse-1



.. image:: https://i.imgur.com/MwnqcLO.png
   :target: https://i.imgur.com/MwnqcLO.png
   :alt: eclipse-1


In the Launcher which started, select the RTI license file and copy it global.

.. image:: https://i.imgur.com/0cQRX04.png
   :target: https://i.imgur.com/0cQRX04.png
   :alt: eclipse-1


Select the install button.

.. image:: https://i.imgur.com/R3eXEc5.png
   :target: https://i.imgur.com/R3eXEc5.png
   :alt: eclipse-1


Install the security package from RTI software.

.. image:: https://i.imgur.com/MJSELif.png
   :target: https://i.imgur.com/MJSELif.png
   :alt: eclipse-1


And the openssl package.

.. image:: https://i.imgur.com/4IH3Jig.png
   :target: https://i.imgur.com/4IH3Jig.png
   :alt: eclipse-1


Unpack the openssl-1.0.2n package and copy it to the RTI install directory.
source /home/ros/rti_connext_dds-5.3.1/resource/scripts/rtisetenv_x64Linux3gcc5.4.0.bash in a console and export RMW_IMPLEMENTATION=rmw_connext_cpp.

Close eclipse-IDE and open it from the shell we sourced all the scripts from above.
Open Project->Preferences in Eclipse and go to Environment.


.. image:: https://i.imgur.com/lzL0vra.png
   :target: https://i.imgur.com/lzL0vra.png
   :alt: eclipse-1


In Eclipse enter environment variables.
The value of the variables can be obtained by running "env > /tmp/out", then source the ROS 2 local_setup.bash, then "env > /tmp/out1" and "diff /tmp/out /tmp/out1".
The difference in that output is what should be entered into Eclipse environment variables so that Eclipse knows about e.g. the new PATH variable.

.. image:: https://i.imgur.com/D30l1Ps.png
   :target: https://i.imgur.com/D30l1Ps.png
   :alt: eclipse-1



.. image:: https://i.imgur.com/ydPADre.png
   :target: https://i.imgur.com/ydPADre.png
   :alt: eclipse-1


Go to Builders and click the "New" button.

.. image:: https://i.imgur.com/GFZXHPb.png
   :target: https://i.imgur.com/GFZXHPb.png
   :alt: eclipse-1


Enter the ament.py settings.

.. image:: https://i.imgur.com/30mWuIF.png
   :target: https://i.imgur.com/30mWuIF.png
   :alt: eclipse-1


Unselect CDT-Builder.

.. image:: https://i.imgur.com/LuwaGBa.png
   :target: https://i.imgur.com/LuwaGBa.png
   :alt: eclipse-1


Go to C++ Build and delete the build command make, because we'll use ament.py.

.. image:: https://i.imgur.com/KiXiAPP.png
   :target: https://i.imgur.com/KiXiAPP.png
   :alt: eclipse-1


Now right-click and run "Build Project".

HINT: if you see the error
error: NDDSHOME set to  but could neither find all optimized libraries nor all debug libraries
Delete the /home/ros/rti_connext_dds-5.3.1/lib/x64Linux3gcc5.4.0/5.3.1/5.3.1 directory with duplicated libraries.


.. image:: https://i.imgur.com/30xv4ka.png
   :target: https://i.imgur.com/30xv4ka.png
   :alt: eclipse-1


Now open two consoles, source ros2_ws/install/local_setup.bash in both consoles and run talker and listener

.. image:: https://i.imgur.com/5NDrDVL.png
   :target: https://i.imgur.com/5NDrDVL.png
   :alt: eclipse-1


In the Eclipse Project, go to git-repositories-view and import local repo.

.. image:: https://i.imgur.com/e0x2dnI.png
   :target: https://i.imgur.com/e0x2dnI.png
   :alt: eclipse-1


Select our directory and select the repository we are interested in seeing.
This will allow us to get git information about those repositories.

.. image:: https://i.imgur.com/RkXnmjr.png
   :target: https://i.imgur.com/RkXnmjr.png
   :alt: eclipse-1


After adding the git repositories to the git-repository-view, right-click on it and select "Import projects".

.. image:: https://i.imgur.com/KxS9x66.png
   :target: https://i.imgur.com/KxS9x66.png
   :alt: eclipse-1


The import source is the directory of our project.

.. image:: https://i.imgur.com/L4HSOEl.png
   :target: https://i.imgur.com/L4HSOEl.png
   :alt: eclipse-1


On the left hand side in the Project Explorer view we can see that this project is next to our ros2_ws project.
Both use the same files, but one is linked with egit and can show git information, while the other cannot.
Open the files from the project which is linked to Egit.

.. image:: https://i.imgur.com/2jBRVlV.png
   :target: https://i.imgur.com/2jBRVlV.png
   :alt: eclipse-1


Below a file linked with Egit is shown.
Right-click beside the line number in the editor and choose "Show Revision information" from the pop-up dialog, then you can see commit information if you hover over it with the mouse.

.. image:: https://i.imgur.com/TyOQFhl.png
   :target: https://i.imgur.com/TyOQFhl.png
   :alt: eclipse-1
