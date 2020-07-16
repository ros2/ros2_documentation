.. redirect-from::

    Set-up-a-new-macOS-CI-node

How to setup a macOS Jenkins node
=================================

.. contents:: Table of Contents
   :depth: 1
   :local:


Install macOS Mojave
--------------------

Install: APFS case-sensitive
Post-install: No Siri, no location services, no cloud anything, no analytics, no filevault disk encryption.

Setup SSH/VNC for Remote Access
-------------------------------

Make sure you don't use too long of a password.
That makes VNC auth fail in bizarre ways.
In particular, VNC auth limits you to 8 characters.


* Go to: Apple->System Preferences->Sharing

  * set hostname to something reasonable
  * check "Remote Login"
  * check "Screen Sharing"

* add Terminal to the dock
* Go to: Apple->System Preferences->Energy Saver

  * set sleep to never
  * uncheck everything

* Go to: Apple->Security

  * click lock to unlock it
  * "Allow Apps from app store and verified developers"
  * uncheck "Require password after 5 minutes" box
  * uncheck "Disable automatic login"

Host Setup
----------

Install XCode tools:

.. code-block:: bash

   $ xcode select --install

Install JDK for Jenkins.
Easiest way is to type ``java`` at the terminal and let Apple link you to Oracle’s JDK download.
I installed the latest JDK 8 (withholding Java 9 for now).

Install Homebrew following instructions at https://brew.sh

Install ``ssh-askpass`` via homebrew

.. code-block:: bash

   $ brew tap theseal/ssh-askpass
   $ brew install ssh-askpass

Create ``~/.bash_profile`` with this one line:

.. code-block:: bash

   . ~/.bashrc

Create ``~/.bashrc`` with one line:

.. code-block:: bash

   export ROS_DOMAIN_ID=XXX  # where XXX is chosen from this document

Set up dummy git names:

.. code-block:: bash

   $ git config --global user.email "nobody@osrfoundation.org"
   $ git config --global user.name "HOSTNAME"

Install ROS 2 Dependencies
--------------------------

Install them according to `our install instructions <../Installation/macOS-Development-Setup>`.

Including:


* brew packages
* pip packages
* the optional RTI Connext and OpenSplice packages
* everything but downloading the source and building ROS 2 (unless you want to do so for testing the setup)

RQt dependencies
~~~~~~~~~~~~~~~~

* ``brew install sip pyqt5``
* Fix some path names when looking for sip stuff during install (see `ROS 1 wiki <https://wiki.ros.org/kinetic/Installation/OSX/Homebrew/Source#Qt_naming_issue>`_):

  ``ln -s /usr/local/share/sip/Qt5 /usr/local/share/sip/PyQt5``

* ``brew install graphviz``
* ``python3 -m pip install pygraphviz pydot``
* ``brew link --force qt``

  This is the quickest solution but may cause issues when upgrading Qt or if other packages are expecting Qt 4.
  Another option is to update your ``PATH`` and ``CMAKE_PREFIX_PATH`` to include the Qt install location:

  .. code-block:: bash

     $ export PATH="$(brew --prefix qt)/bin:$PATH"
     $ export CMAKE_PREFIX_PATH="$(brew --prefix qt):$CMAKE_PREFIX_PATH"

RTI Connext Specific Instructions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


* The Open Robotics license is here (private repo): https://github.com/osrf/rticonnextdds-src/blob/license/rti_license.dat
* Open the RTI launcher application

  * In the RTI launcher, open the file dialog to choose the license file.
  * Install it for all users.
  * Click the Installation tab
  * Click RTI Package installer

* Navigate to the connext extracted directory (usually something like ``/Applications/rti_...``

  * Select the ``rti_security rtipkg`` (don’t bother with the openssl ones, we use system openssl)

* Set the shared memory parameters from https://community.rti.com/kb/osx510

  * Do not bother to reboot yet.

* Download ``openssl-1.0.2n-target-x64Darwin17clang9.0.tar.gz`` from RTI's website using an RTI account and extract it.

.. code-block:: bash

   tar -xzvf openssl-1.0.2n-target-x64Darwin17clang9.0.tar.gz

* Add environment variables pointing to the paths to the RTI openssl libraries.

.. code-block:: bash

   export RTI_OPENSSL_BIN=/Users/osrf/openssl-1.0.2n/x64Darwin17clang9.0/release/bin
   export RTI_OPENSSL_LIBS=/Users/osrf/openssl-1.0.2n/x64Darwin17clang9.0/release/lib

Setting up the Jenkins Workspace and Agent
------------------------------------------

.. code-block:: bash

   $ mkdir jenkins jenkins-agent
   $ cd jenkins-agent
   $ wget https://ci.ros2.org/jnlpJars/slave.jar

Copy the jenkins agent plist from https://gist.github.com/nuclearsandwich/c9546e76ba63767bc1025c393e85235b

Edit the file to match the jnlp URL and secret of the host you’re setting up.
You may need to create a new agent if you’re not re-imaging an existing one.

.. code-block:: bash

   $ mkdir ~/Library/LaunchAgents
   $ cp ~/jenkins-agent/org.ros2.ci.jenkins-agent.plist ~/Library/LaunchAgents
   $ launchctl load -w ~/Library/LaunchAgents/org.ros2.ci.jenkins-agent.plist

Reboot! You should be good to go, run some test CI jobs.
