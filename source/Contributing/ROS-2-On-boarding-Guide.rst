.. redirect-from::

   ROS-2-On-boarding-Guide

ROS 2 on-boarding guide
=======================

The purpose of this guide is to supplement the on-boarding of new developers when they join the ROS 2 team.
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

Your GitHub username must be added with the same permissions as existing users to Jenkins (https://ci.ros2.org/configureSecurity/\ ).
This can be done by any existing user.

How to access the machines running the ci.ros2.org?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Only do this if you're working at OSRF or if you're asked to log into the machines.
To be able to ssh into the node hosted on AWS, you need give request access from Tully Foote (tfoote@osrfoundation.org).

Request access to the Google drive ROS 2 folder
-----------------------------------------------

Only do this if you're working at OSRF or need access to a particular document.
To request access send an email to ros@osrfoundation.org (anybody on the mailing list can share it).

Choose a DDS domain ID
----------------------

ROS 2 uses DDS as the underlying transport and DDS supports a physical segmentation of the network based on the "domain ID" (it is used to calculate the multicast port).
We use a unique value for this on each machine (or group of machines) to keep each group's ROS 2 nodes from interfering with other developers' testing.
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

We track open tickets and active PRs related to upcoming releases and larger projects using `GitHub project boards <https://github.com/orgs/ros2/projects>`_.

The usual workflow is (this list is a work in progress):

* Discuss design (GitHub ticket, and a meeting if needed)
* Assign implementation to a team member
* Write implementation on a feature branch

  * Please check out the `developer guide <Developer-Guide>` for guidelines and best practices

* Write tests
* Enable and run linters
* Run tests locally using ``colcon test`` (see `colcon tutorial <../Tutorials/Colcon-Tutorial>`)
* Once everything builds locally without warnings and all tests are passing, run CI on your feature branch:

  * Go to ci.ros2.org
  * Log in (top right corner)
  * Click on the ``ci_launcher`` job
  * Click "Build with Parameters" (left column)
  * In the first box "CI_BRANCH_TO_TEST" enter your feature branch name
  * Hit the ``build`` button

* If your use case requires running code coverage:

  * Go to ci.ros2.org
  * Log in (top right corner)
  * Click on the ``ci_linux_coverage`` job
  * Click "Build with Parameters" (left column)
  * Be sure of leaving "CI_BUILD_ARGS" and "CI_TEST_ARGS" with the default values
  * Hit the ``build`` button
  * At the end of the document there are instructions on how to :ref:`interpret the result of the report <read-coverage-report>` and :ref:`calculate the coverage rate <calculate-coverage-rate>`

* If the CI job built without warnings, errors and test failures, post the links of your jobs on your PR or high-level ticket aggregating all your PRs (see example `here <https://github.com/ros2/rcl/pull/106#issuecomment-271119200>`__)

  * Note that the markdown for these badges is in the console output of the ``ci_launcher`` job

* To get the PR reviewed, you need to put the label "in review":

  * Through GitHub interface:

    * Click on "" next to labels
    * Remove "in progress" label if applicable
    * Add "in review" label

  * If the PR is part of a project board:

    * Drag the card from "In progress" to "In review"

* When the PR has been approved:

  * the person who submitted the PR merges it using "Squash and Merge" option so that we keep a clean history

    * If the commits deserve to keep separated: squash all the nitpick/linters/typo ones together and merge the remaining set

      * Note: each PR should target a specific feature so Squash and Merge should make sense 99% of the time

* Delete the branch once merged

GitHub tips
^^^^^^^^^^^

Link PRs to the issues they address using `keywords <https://help.github.com/en/github/managing-your-work-on-github/linking-a-pull-request-to-an-issue#linking-a-pull-request-to-an-issue-using-a-keyword>`_ and the ticket number.
This will close the issue once the pull request is merged.

* In the same repo: "fixes #216"
* In another repo: "fixes ros2/rosidl#216"

Build Farm Introduction
-----------------------

The build farm is located at `ci.ros2.org <https://ci.ros2.org/>`__.

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

There are several categories of jobs on the buildfarm:


* manual jobs (triggered manually by developers):

  * ci_linux: build + test the code on Ubuntu Xenial
  * ci_linux-aarch64: build + test the code on Ubuntu Xenial on an ARM 64-bit machine (aarch64)
  * ci_linux_coverage: build + test + generation of test coverage
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

  * Repeated: build then run each test up to 20 times or until failed (aka flakiness hunter)

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

Note on Coverage runs
^^^^^^^^^^^^^^^^^^^^^

ROS 2 packages are organized in a way that the testing code for a given package is not only contained within the package, but could also be present in a different package.
In other words: packages can exercise code belonging to other packages during the testing phase.

To achieve the coverage rate reached by all code available in the ROS 2 core packages it is recommended to run builds using a fixed set of proposed repositories.
That set is defined in the default parameters of coverage jobs in Jenkins.


.. _read-coverage-report:

How to read the coverage rate from the buildfarm report
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To see the coverage report for a given package:

* When the ``ci_linux_coverage`` build finishes, click on ``Coverage Report``
* Scroll down to the ``Coverage Breakdown by Package`` table
* In the table, look at the first column called "Name"

The coverage reports in the buildfarm include all the packages that were used in the ROS workspace.
The coverage report includes different paths corresponding to the same package:

* Name entries with the form: ``src.*.<repository_name>.<package_name>.*``
  These correspond to the unit test runs available in a package against its own source code
* Name entries with the form: ``build.<repository_name>.<package_name>.*``
  These correspond to the unit test runs available in a package against its files generated at building or configuring time
* Name entries with the form: ``install.<package_name>.*``
  These correspond to the system/integration tests coming from testing runs of other packages

.. _calculate-coverage-rate:

How to calculate the coverage rate from the buildfarm report
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Get the combined unit coverage rate using the automatic script:

 * From the ci_linux_coverage Jenkins build copy the URL of the build
 * Download the `get_coverage_ros2_pkg <https://raw.githubusercontent.com/ros2/ci/master/tools/get_coverage_ros2_pkg.py>`__ script
 * Execute the script: ``./get_coverage_ros2_pkg.py <jenkins_build_url> <ros2_package_name>`` (`README <https://github.com/ros2/ci/blob/master/tools/README.md>`__)
 * Grab the results from the "Combined unit testing" final line in the output of the script

Alternative: get the combined unit coverage rate from coverage report (require manual calculation):

* When the ci_linux_coverage build finishes, click on ``Cobertura Coverage Report``
* Scroll down to the ``Coverage Breakdown by Package`` table
* In the table, under the first column "Name", look for (where <package_name> is your package under testing):

  * all the directories under the pattern ``src.*.<repository_name>.<package_name>.*`` grab the two absolute values in the column "Lines".
  * all the directories under the pattern ``build/.<repository_name>.*`` grab the two absolute values in the column "Lines".

* With the previous selection: for each cell, the first value is the lines tested and the second is the total lines of code.
  Aggregate all rows for getting the total of the lines tested and the total of lines of code under test.
  Divide to get the coverage rate.

.. _measure-coverage-locally:

How to measure coverage locally using lcov (Ubuntu)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To measure coverage on your own machine, install ``lcov``.
.. code-block:: bash

     sudo apt install -y lcov

The rest of this section assumes you are working from your colcon workspace.
Compile in debug with coverage flags.
Feel free to use colcon flags to target specific packages.

.. code-block:: bash

     colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS} --coverage" -DCMAKE_C_FLAGS="${CMAKE_C_FLAGS} --coverage"

``lcov`` requires an initial baseline, which you can produce with the following command.
Update the output file location for your needs.

.. code-block:: bash

     lcov --no-external --capture --initial --directory . --output-file ~/ros2_base.info

Run tests for the packages that matter for your coverage measurements.
For example, if measuring ``rclcpp`` also with ``test_rclcpp``

.. code-block:: bash

     colcon test --packages-select rclcpp test_rclcpp

Capture the lcov results with a similar command this time dropping the ``--initial`` flag.

.. code-block:: bash

     lcov --no-external --capture --directory . --output-file ~/ros2.info

Combine the trace .info files:

.. code-block:: bash

     lcov --add-tracefile ~/ros2_base.info --add-tracefile ~/ros2.info --output-file ~/ros2_coverage.info

Generate html for easy visualization and annotation of covered lines.

.. code-block:: bash

    mkdir -p coverage
    genhtml ~/ros2_coverage.info --output-directory coverage


Learning ROS 2 concepts at a high level
---------------------------------------

All ROS 2 design documents are available at https://design.ros2.org/ and there is some generated documentation at https://docs.ros2.org/.
