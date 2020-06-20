.. redirect-from::

    Set-up-a-new-Windows-CI-node

How to setup a Windows Jenkins node
===================================

.. contents:: Table of Contents
   :depth: 1
   :local:

.. note::

   See this older (private) document for previous instructions:

   https://docs.google.com/document/d/1SmmWa7MVnwjmMw9XJF33-fsa0dtkYj2AeEXBa8BCsYs/edit

Install and Update Windows 10
-----------------------------

We use the normal Windows 10 version, not the enterprise, but other than that we just do system updates and use default settings otherwise.

Install Dependencies for ROS 2
------------------------------

Follow our Windows "from source" `installation instructions <../Installation/Windows-Development-Setup>`.

Setup git
---------

Ensure that the git installation has a (garbage) email and name, otherwise it will fail if it ever needs to make merge commits when merging branches with "master".

Note that this must be done in the context of the "System" user, which is what the Jenkins service will run as.


* Become the system user by using ``psexec``:

  * Download it from: https://technet.microsoft.com/en-us/sysinternals/bb897553.aspx
  * Then extract the zip, open a command-prompt as administrator, and run: ``psexec -i -s cmd.exe``

This is all pieced together from a couple of pages here:

* http://blog.thomasvandoren.com/jenkins-windows-slave-with-git.html
* https://answers.atlassian.com/questions/128324/where-is-the-home-directory-for-the-system-user

Once you are the system user, set the git config:

.. code-block:: bash

   > git config --global user.email "noreply@osrfoundation.org"
   > git config --global user.name "nobody"

Setup Jenkins Agent
-------------------

Download and install Java from Oracle:

https://java.com/en/download/

Create the ``C:\J`` folder.

Go to https://ci.ros2.org and select: Manage Jenkins -> Manage Nodes -> New node

Copy an existing node and choose a working Windows node (at the time of the writing Windshield or Portable), fill in all the description fields, set the label to “windows slave” and save.

Hit the "launch" button to download the "Java Web Start App" and save it on the desktop.

Open an Administrator ``cmd.exe`` and then:

.. code-block:: bash

   > cd \path\to\downloaded\file
   > .\slave-agent.jnlp

Once open, go to File->"Install as a Service".
Then you can close the slave agent as it will start on boot.

Install RTI Connext
-------------------

Download and install Connext binary:

http://www.rti.com/downloads/connext-files.html

Make sure to also install the security plugins.

Download the license file from our private GitHub repository:

https://github.com/osrf/rticonnextdds-src/tree/30adec34dbaf1445914ff9e003640b0c50ab9969

Run the RTI launcher application and point it at the license file.

Finally, just need to reboot to be sure the node comes online as expected.
