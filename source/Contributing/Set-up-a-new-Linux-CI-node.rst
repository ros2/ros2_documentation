.. redirect-from::

    Set-up-a-new-Linux-CI-node

How to setup Linux Jenkins nodes
================================

.. contents:: Table of Contents
   :depth: 1
   :local:

This page describes how to set up a Linux machine for ROS 2 CI jobs using AWS.

Creating an AWS instance
------------------------

In short, use the company AWS account to launch an instance running based off the official Ubuntu 16.04 AMI.


* **AMI:** Ubuntu 16.04
* **Region:** N. California us-west-1a
* **Type:** c4.large
* **Storage:** EBS 1TB
* **Security Group:** ``ROS 2 Jenkins Build Machines``
* **Key pair:** Create a new pair with a descriptive name like ``ci_ros2_linux_4``

  * Make sure to save it with the other credentials so others can access this machine

Give the instance a descriptive name like ``ROS 2 CI (linux 4)``.
Record the IP address `here <https://docs.google.com/spreadsheets/d/1OSwqbE3qPF8v3HSMr8JOaJ6r4QOiQFk6pwgaudXVE-4/edit#gid=0>`__ (private).

Setting up the machine
----------------------

In short, make sure the jenkins master can ssh into the new node and run docker.


#. Use the key pair to log into the new node

   * ``ssh -i ci_ros2_linux_4.pem ubuntu@IPADDRESS``

#.
   Run the following commands

   .. code-block:: bash

       sudo apt update
       sudo apt install -y git
       sudo apt install -y openjdk-8-jre-headless
       sudo bash -c 'echo "deb http://repositories.ros.org/ubuntu/testing/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
       sudo bash -c 'curl --silent https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc |sudo apt-key add -'
       sudo apt update
       sudo apt install -y python-vcstool
       curl -fsSL https://get.docker.com/ | sh
       sudo adduser --disabled-password jenkins
       sudo usermod -aG docker jenkins
       sudo service docker start

#.
   Make sure the jenkins user can run docker

   .. code-block:: bash

       sudo su jenkins
       docker run hello-world

#. As the jenkins user, add the master's public key to ``authorized_keys``

   #. SSH into the jenkins master and get the contents of the public key (probably at /var/lib/jenkins/.ssh/id_rsa.pub, or wherever the home directory of the jenkins user is)
   #. SSH into the new slave and add that key to the authorized keylist
      .. code-block:: bash

         # on new slave
         cd /home/jenkins/
         mkdir .ssh
         chmod 700 .ssh
         touch .ssh/authorized_keys
         chmod 600 .ssh/authorized_keys
         # Paste id_rsa.pub from the jenkins master into this file
         vim .ssh/authorized_keys

Adding it to the master
-----------------------


#. Add a new agent to https://ci.ros2.org/computer/

   * **Number of executors:** 1
   * **Remote root directory:** /home/jenkins
   * **Labels:** ``linux``
   * **Launch method:** Launch slave agents via ssh

     * **Host:** Ip address of new node
     * **Credentials:** ``Jenkins``
     * **Host Key Verification Strategy:** ``Manually provided key verification strategy``

       * **SSH Key** paste the contents of ``/etc/ssh/ssh_host_rsa_key.pub`` from the new node here.

     * **Node Properties:**

       * Check ``Notify when Node online status changes`` and set the email to the ros2 buildfarm google group.

#. Launch the agent on the new node
