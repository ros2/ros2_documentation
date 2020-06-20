.. redirect-from::

    CI-Server-Setup

How to setup the Jenkins master
===============================

.. contents:: Table of Contents
   :depth: 2
   :local:

Installing
----------

Install the latest LTS release from http://pkg.jenkins-ci.org/debian-stable/

Running on port 80
------------------

Use this SO answer to setup a subdomain to a port:

http://serverfault.com/a/140161/186748

(Removed the ``hudson`` in each of the lines that contained it.)

Temporary rewrite for changed job name
--------------------------------------

We renamed some of the jobs, so here are some rewrite rules in Apache (``/etc/apache2/sites-enabled/ci.ros2.org.conf``):

.. code-block:: bash

   # Temporary rewrite rule because we changed the Windows job name.
   RewriteEngine On
   RewriteRule ^(.*)/ros2_batch_ci_linux/(.*)$ $1/ci_linux/$2 [R=301,L]
   RewriteRule ^(.*)/ros2_batch_ci_osx/(.*)$ $1/ci_osx/$2 [R=301,L]
   RewriteRule ^(.*)/ros2_batch_ci_windows/(.*)$ $1/ci_windows_opensplice/$2 [R=301,L]
   RewriteRule ^(.*)/ros2_batch_ci_windows_opensplice/(.*)$ $1/ci_windows_opensplice/$2 [R=301,L]
   RewriteRule ^(.*)/ros2_batch_ci_windows_connext_static/(.*)$ $1/ci_windows_connext_static/$2 [R=301,L]
   RewriteRule ^(.*)/ros2_batch_ci_windows_connext_dynamic/(.*)$ $1/ci_windows_connext_dynamic/$2 [R=301,L]
   RewriteRule ^(.*)/ci_windows_opensplice/(.*)$ $1/ci_windows/$2 [R=301,L]
   RewriteRule ^(.*)/ci_windows_connext_static/(.*)$ $1/old_windows_connext_static/$2 [R=301,L]
   RewriteRule ^(.*)/ci_windows_connext_dynamic/(.*)$ $1/old_windows_connext_dynamic/$2 [R=301,L]
   RewriteRule ^(.*)/ci_windows_fastrtps/(.*)$ $1/old_windows_fastrtps/$2 [R=301,L]

   RewriteRule ^(.*)/ros2_batch_ci_linux_nightly/(.*)$ $1/nightly_linux/$2 [R=301,L]
   RewriteRule ^(.*)/ros2_batch_ci_osx_nightly/(.*)$ $1/nightly_osx/$2 [R=301,L]
   RewriteRule ^(.*)/ros2_batch_ci_windows_opensplice_nightly/(.*)$ $1/nightly_windows_opensplice/$2 [R=301,L]
   RewriteRule ^(.*)/ros2_batch_ci_windows_connext_static_nightly/(.*)$ $1/nightly_windows_connext_static/$2 [R=301,L]
   RewriteRule ^(.*)/ros2_batch_ci_windows_connext_dynamic_nightly/(.*)$ $1/nightly_windows_connext_dynamic/$2 [R=301,L]
   RewriteRule ^(.*)/nightly_windows_opensplice/(.*)$ $1/nightly_windows/$2 [R=301,L]
   RewriteRule ^(.*)/nightly_windows_connext_static/(.*)$ $1/old_night_windows_connext_static/$2 [R=301,L]
   RewriteRule ^(.*)/nightly_windows_connext_dynamic/(.*)$ $1/old_night_windows_connext_dynamic/$2 [R=301,L]

   RewriteRule ^(.*)/ros2_packaging_linux/(.*)$ $1/packaging_linux/$2 [R=301,L]
   RewriteRule ^(.*)/ros2_packaging_osx/(.*)$ $1/packaging_osx/$2 [R=301,L]
   RewriteRule ^(.*)/ros2_packaging_windows_opensplice/(.*)$ $1/packaging_windows_opensplice/$2 [R=301,L]
   RewriteRule ^(.*)/packaging_windows_opensplice/(.*)$ $1/packaging_windows/$2 [R=301,L]

Install stuff (needed on master and slaves)
-------------------------------------------

.. code-block:: bash

   sudo apt update
   sudo apt install -y git
   # Your java version will vary depending on your OS:
   #sudo apt install openjdk-7-jre-headless
   #sudo apt install openjdk-8-jre-headless
   # For ARM native servers, we need the tomcat native libs to support ssh-agent
   # (https://issues.jenkins-ci.org/browse/JENKINS-30746)
   #sudo apt install libtcnative-1
   # qemu and vcs are required for ARM builds
   sudo apt install -y qemu-user-static
   sudo bash -c 'echo "deb http://repositories.ros.org/ubuntu/testing/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo bash -c 'curl --silent https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc |sudo apt-key add -'
   # Or, on aarch64:
   #sudo apt install docker.io
   sudo apt update
   sudo apt install -y python-vcstool
   curl -fsSL https://get.docker.com/ | sh
   sudo adduser --disabled-password jenkins
   sudo usermod -aG docker jenkins
   sudo service docker start

Adding a Linux slave to the farm
--------------------------------

Approximately:


* Shell into the master (``ci.ros2.org``), copy ``/var/lib/jenkins/.ssh/id_rsa.pub`` and paste it into ``/home/jenkins/.ssh/authorized_keys`` on the new machine.
* Copy config from the ``linux 2`` machine, rename and otherwise modify as needed (e.g., change the IP/host).
* Copy ``/etc/ssh/ssh_host_rsa_key.pub`` from the new machine and add it as an entry in ``/var/lib/jenkins/.ssh/known_hosts`` (with the new machine's IP) on the master, then re-hash that file on the master: ``ssh-keygen -H``.

Configuring Jenkins
-------------------

First update all the preinstalled plugins.

Authentication
^^^^^^^^^^^^^^

Setup authentication with the ``github-oauth`` plugin.
Install and follow their setup instructions:

https://wiki.jenkins-ci.org/display/JENKINS/Github+OAuth+Plugin

Create an application entry on the ros2 GitHub organization:

https://github.com/organizations/ros2/settings/applications/215300

Tune the permissions in ``Manage Jenkins->Configure Global Security``.

Plugins
^^^^^^^

Install these plugins:


* ``ansicolor``
* ``description-setter``
* ``github`` (other git* plugins are deps of the ``github-oauth`` plugin)
* ``greenballs``
* ``groovy``
* ``parameterized-trigger``
* ``PrioritySorter``
* ``jobrequeue``
* ``ssh-agent``
* ``warnings``
* ``xunit``

Adding an ssh key
^^^^^^^^^^^^^^^^^

Jenkins needs a valid ssh key in order to pull from some of our private repositories, for example to get the rti deb files.

So, let's create an ssh key for the jenkins user on the webserver:

.. code-block:: bash

   sudo su jenkins
   cd
   mkdir .ssh
   ssh-keygen -t rsa

Now add to the jenkins credentials as an "From the jenkins master ~/.ssh" with the user id of ``ros2-buildfarm``.

Add this key to a "machine" GitHub account created for this farm and add that user, ``ros2-buildfarm``, to the ``ros2``, ``ament``, and ``osrf`` organizations.

Creating Jobs
-------------

Clone the ``ros2/ci`` repository to the default branch (``master``):

.. code-block:: bash

   git clone https://github.com/ros2/ci.git

Clone the ``ros_buildfarm`` repository:

.. code-block:: bash

   git clone https://github.com/ros-infrastructure/ros_buildfarm.git

Install the ``jenkinsapi`` and ``EmPy`` Python packages:

.. code-block:: bash

   sudo apt install python3-pip
   sudo -H python3 -m pip install -U pip
   sudo -H python3 -m pip install jenkinsapi EmPy

Then setup auth:

.. code-block:: bash

   mkdir -p ~/.buildfarm
   vim ~/.buildfarm/jenkins.ini

Put this in the ``jenkins.ini`` file:

.. code-block:: bash

   [http://ci.ros2.org]
   username=wjwwood
   password=<your application token>

Now, you should first login with GitHub on Jenkins if you haven't already.
Then put your GitHub username in and for the application token, browse to the configuration of your user on Jenkins:

https://ci.ros2.org/user/wjwwood/configure

In those settings there should be a field called API Token.
Copy that field for your password.

Now you can create the jobs:

.. code-block:: bash

   $ PYTHONPATH=`pwd`/../ros_buildfarm ./create_jenkins_job.py -u http://ci.ros2.org
   Connecting to Jenkins 'http://ci.ros2.org'
   Connected to Jenkins version '1.617'
   Creating job 'ros2_batch_ci_windows'
   The Jenkins master does not require a crumb
   Creating job 'ros2_batch_ci_osx'
   Creating job 'ros2_batch_ci_linux'
   Creating job 'ros2_batch_ci_launcher'

Tuning Auto-generated Jobs
^^^^^^^^^^^^^^^^^^^^^^^^^^

The final step is to fine tune the jobs.
For each job you'll want to check the ssh key being used for the git clone (only on Linux) and the ssh-agent.
It should be set to the ssh key setup in the previous steps for the jenkins user.

I also updated the slaves which the jobs will run on to make sure they matched the names of the slaves  added for Linux, macOS and Windows.

Disk space
----------

Over time docker images and particularly containers will pile up.

To clean up use:

.. code-block:: bash

   docker rm $(docker ps -a -q)
   docker rmi $(docker images -q -f dangling=true)

from https://www.calazan.com/docker-cleanup-commands/
