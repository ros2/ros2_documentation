
ROS 2 On-boarding Guide
=======================

The purpose of this guide is supplement the on-boarding of new developers when they join the ROS 2 team.
It is mostly used by the ROS 2 team, but it might be useful for others as well.

Request access to the GitHub organizations
------------------------------------------

Our code is federated across a few GitHub organizations, you'll want access to them so you can make pull requests with branches rather than forks:


* https://github.com/ros2
* https://github.com/ament
* https://github.com/osrf (optional, as-needed)

Request access to the buildfarm
-------------------------------

The build farm is hosted at: ci.ros2.org

To request access send an email to ros@osrfoundation.org.

How to give access?
^^^^^^^^^^^^^^^^^^^

Your GitHub username must be added with the same permissions as existing users to Jenkins (http://ci.ros2.org/configureSecurity/\ ).
This can be done by any existing user.

How to access the machines running the ci.ros2.org?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Only do this if you're working at OSRF or if you're asked to log into the machines.
To be able to ssh into the node hosted on AWS, you need give request access from Tully Foote (tfoote@osrfoundation.org).

Request access to the Google drive ROS2 folder
----------------------------------------------

Only do this if you're working at OSRF or need access to a particular document.
To request access send an email to ros@osrfoundation.org (anybody on the mailing list can share it).

Choose a DDS domain ID
----------------------

ROS2 uses DDS as the underlying transport and DDS supports a physical segmentation of the network based on the "domain ID" (it is used to calculate the multicast port.
We use a unique value for this on each machine to keep our ROS2 nodes from interfering from each other.
We expose this setting via the ``ROS_DOMAIN_ID`` environment variable and use a document to ensure we don't accidentally choose the same one as someone else.
This is, however, only important for people who will be working on the OSRF network, but it isn't a bad idea to set up at any organization with multiple ROS 2 users on the same network.

Get a Personal ROS_DOMAIN_ID
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Go to the `ROS 2 Assigned Domain ID's Spreadsheet <https://docs.google.com/spreadsheets/d/1YuDSH1CeySBP4DaCX4KoCDW_lZY4PuFWUu4MW6Vsp1s/edit>`__ and reserve an ID, or email ros@osrfoundation.org and ask for one to be allocated to you.
Numbers under 128 are preferred.

To ensure it is always set, add this line to your ``~/.bashrc`` or equivalent:

.. code-block:: bash

   export ROS_DOMAIN_ID=<your_domain_id>

Watching ROS 2 Repositories
---------------------------

We try to spread our responsibilities out across the team and so we ask everyone to watch the main repositories for ROS 2.


* What am I currently watching?

  * https://github.com/watching

* How do I watch a repository?

  * https://help.github.com/articles/watching-repositories/

* Which repositories should I watch?

  * All the repositories listed in the `ros2.repos file <https://github.com/ros2/ros2/blob/master/ros2.repos>`__, included the commented out ones,
  * Also all of these extra repositories from the ROS 2 organization:

    * https://github.com/ros2/ci
    * https://github.com/ros2/design
    * https://github.com/ros2/ros_astra_camera
    * https://github.com/ros2/ros_core_documentation
    * https://github.com/ros2/ros2
    * https://github.com/ros2/sros2
    * https://github.com/ros2/turtlebot2_demo
    * https://github.com/ros2/joystick_drivers

Developer Workflow
------------------

We track all open tickets and current PRs using waffle.io: https://waffle.io/ros2/ros2

Higher level tasks are tracked on the internal (private) Jira: https://osrfoundation.atlassian.net/projects/ROS2

The usual workflow is (this list is a work in progress):


* Discuss design (GitHub ticket, and a meeting if needed)
* Assign implementation to a team member
* Write implementation on a feature branch

  * Please check out the `developer guide <Developer-Guide>` for guidelines and best practices

* Write tests
* Enable and run linters
* Run tests locally using ``colcon test`` (see `colcon tutorial <../Tutorials/Colcon-Tutorial>`\ )
* Once everything builds locally without warnings and all tests are passing, run CI on your feature branch:

  * Go to ci.ros2.org
  * Log in (top right corner)
  * Click on the ``ci_launcher`` job
  * Click "Build with Parameters" (left column)
  * In the first box "CI_BRANCH_TO_TEST" enter your feature branch name
  * Hit the ``build`` button

* If built without warnings, errors and test failures, post the links of your jobs on your PR or high level ticket aggregating all your PRs (example `here <https://github.com/ros2/rcl/pull/106#issuecomment-271119200>`__\ )

  * Note that the markdown for these badges is in the console output of the ``ci_launcher`` job

* To get the PR reviewed, you need to put the label "in review":

  * Through github interface:

    * Click on "" next to labels
    * Remove "in progress" label if applicable
    * Add "in review" label

  * Through waffle:

    * Drag your PR to the "in review" column

* When the PR has been approved:

  * the person who submitted the PR merges it using "Squash and Merge" option so that we keep a clean history

    * If the commits deserve to keep separated: squash all the nitpick/linters/typo ones together and merge the remaining set

      * Note: each PR should target a specific feature so Squash and Merge should make sense 99% of the time

* Delete the branch once merged

Waffle.io How-to
----------------

Here are some tips on how to use our Kanban board on waffle.io:


* Assigning labels: drag and drop cards to the column with the label you want to assign
* Connecting Issues/PR: Waffle allows to connect cards together using keywords

  * Note1: The keywords need to be placed in the 1st comment of the GitHub ticket
  * Note2: Waffle uses the "simplified" GitHub reference and not the full URL to connect card.

    * Does not work:

      * "connects to https://github.com/ros2/rosidl/issues/216"

    * Works:

      * In the same repo: "connects to #216"
      * In another repo: "connects to ros2/rosidl#216"

Build Farm Introduction
-----------------------

The build farm is located at `ci.ros2.org <http://ci.ros2.org/>`__.

Every night we run nightly jobs which build and run all the tests in various scenarios on various platforms.
Additionally, we test all pull requests against these platforms before merging.

This is the current set of target platforms and architectures, though it evolves overtime:


* Ubuntu 16.04 Xenial

  * amd64
  * aarch64

* macOS 10.12 Sierra

  * amd64

* Windows 10

  * amd64

There several categories of jobs on the buildfarm:


* manual jobs (triggered manually by developers):

  * ci_linux: build + test the code on Ubuntu Xenial
  * ci_linux-aarch64: build + test the code on Ubuntu Xenial on an ARM 64-bit machine (aarch64)
  * ci_osx: build + test the code on MacOS 10.12
  * ci_windows: build + test the code on Windows 10
  * ci_launcher: trigger all the jobs listed above

* nightly (run every night):

  * Debug: build + test the code with CMAKE_BUILD_TYPE=Debug

    * nightly_linux_debug
    * nightly_linux-aarch64_debug
    * nightly_osx_debug
    * nightly_win_deb

  * Release: build + test the code with CMAKE_BUILD_TYPE=Release

    * nightly_linux_release
    * nightly_linux-aarch64_release
    * nightly_osx_release
    * nightly_win_rel

  * Repeated: build then run each test up to 20 times or until failed (aka flakyness hunter)

    * nightly_linux_repeated
    * nightly_linux-aarch64_repeated
    * nightly_osx_repeated
    * nightly_win_rep

  * Coverage:

    * nightly_linux_coverage: build + test the code + analyses coverage for c/c++ and python

      * results are exported as a cobertura report

* packaging (run every night, against fastrtps; result is bundled into an archive):

  * packaging_linux
  * packaging_osx
  * Packaging_windows

Learning ROS2 concepts at a high level
--------------------------------------

All ROS2 design documents are available at http://design.ros2.org/ and there is some generated documentation at http://docs.ros2.org/
