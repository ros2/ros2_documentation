Launch testing usage
====================

.. contents:: Contents
   :depth: 2
   :local:

Overview
--------
`Launch testing <https://github.com/ros2/launch/tree/master/launch_testing>`__ is intended to be a framework to write launch integration tests. One can spawn nodes  and processes using this framework and can monitor :

* Exit codes of processes
* Whether the processes shutdown properly
* ``stdin, stdout, stderr`` of all processes

Some tests can be run concurrently with launch and can interact with the running processes. Tests can fail when a process dies unexpectedly.

This article assumes the reader is comfortable with the ROS2 launch system, a short tutorial of which can be found `here <https://docs.ros.org/en/foxy/Tutorials/Launch-Files/Creating-Launch-Files.html>`__. The test ``.py`` files are run using the ``launch_test`` utility, typically found in ``ros2_ws/install/launch_testing/bin/launch_test`` , if you’ve installed ros2 from source.

General Template
----------------
Following snippet shows a general template that can be followed when writing test cases. The test functions usually end in assertions (assert, assertEqual, assertInStdout, etc)

.. code-block:: python

   # Imports
   import unittest
   ...
   
   # Nodes / processes to be launched go here
   @pytest.mark.launch_test
   def generate_test_description():
       return launch.LaunchDescription([
           ...
       ])
   
   # Tests in this class will be run at the same time the nodes are launched
   # and are expected to interact with the nodes / processes
   class TestFixture_1(unittest.TestCase):
       def test_1(self, proc_output):
           ...
           assert <something>
   
       def test_2(self, proc_output):
           ...
           assert <something>
   
   # These tests run after the the above TextFixture_1 and after the nodes are shut down
   @launch_testing.post_shutdown_test()
   class TestsPostShutdown(unittest.TestCase):
       def test_3(self, proc_info):
           ...
           assert <something>

The Launch Description
----------------------
The test starts with the generate_test_description function, decorated by @pytest.mark.launch_test , which launches the required nodes / processes. The function should return a ``launch.LaunchDescription`` object that launches the system to be tested. 
The launch description must include a ``ReadyToTest`` action to signal to the test framework that it's safe to start the active tests.
In the snippet below, the description indicates to the framework that it's safe to start the test around the same time the ExecuteProcess is run.  

:: 

  def generate_test_description():
    return launch.LaunchDescription([
      launch.actions.ExecuteProcess(
        cmd=['echo', 'hello_world']
      ),
      launch_testing.actions.ReadyToTest()
    ])

The launch description can optionally include a dummy process ``launch_testing.util.KeepAliveProc()`` to keep the launch service alive while the tests are running. More more information check out `this example. <https://github.com/ros2/launch/blob/f891aed9f904df6397ef554f7e0b36bb37b30529/launch_testing/test/launch_testing/examples/args_launch_test.py#L63>`__

Examples
--------
It is recommended that the reader should go through the simple examples provided in the links below to get a feel of the general code flow.
Since ``launch_testing`` can be used for generic processes and separate from ROS related actions, the exmaples have been split in 2 directories.

* Examples that do not require ROS actions are located `here <https://github.com/ros2/launch/tree/master/launch_testing/test/launch_testing/examples>`__. A brief explanation on them is located `here <https://github.com/ros2/launch/blob/master/launch_testing/README.md>`__
* ROS specific examples are located here <TODO add link>. The README in the directory explains the usage.

Passing arguments to tests
--------------------------

<TODO>

Active Tests
------------

Any classes that inherit from ``unittest.TestCase`` and not decorated with the ``@post_shutdown_test`` descriptor will be run concurrently with the process under test. 
These tests are expected to interact with the running processes in some way. The tests inside a fixture (class) do not run in any specific order. For e.g, in the ``TestFixture_1`` class in the above template, it is not guaranteed that ``test_1`` will run before ``test_2`` or vice versa.

Post Shutdown Tests
-------------------
Any classes that inherit from ``unittest.TestCase`` that are decorated with the ``@post_shutdown_test`` descriptor will be run after the launched processes have been shut down. 
These tests have access to the exit codes and the stdout of all of the launched processes, as well as any data created as a side-effect of running the processes.

Assertions and actions
----------------------
List of available assertions in ``launch_testing`` and their explanations can be found here. <TODO>
Most of the actions in ``launch.actions`` and ``launch_ros.actions`` can be used with launch testing. <TODO : Add links>

PYTHONUNBUFFERED environment variable
-------------------------------------

This environment variable needs to be set sometimes in situations where we need to read from the stdout of a python process (for e.g. when using ``assertWaitForOutput()`` ), as the stdout of python is block buffered when the output is non-interactive. 
Note that currently this works for python processes launched using the interpreter directly ( ``python3 something.py`` ) and not on ``ros2 run pkg exec``. Check out `this example <https://github.com/ros2/launch/blob/master/launch_testing/test/launch_testing/examples/context_launch_test.py#L41>`__  for detailed usage. 

Example usage :

.. code-block:: python

   launch.actions.ExecuteProcess(
	cmd =['python3', 'some_script.py'],
	additional_env={'PYTHONUNBUFFERED': '1'},
	output='screen'
   )

Handler objects 
---------------
The launch_testing framework automatically adds some member fields to each test case so that the tests can access process output and exit codes.

* ``self.proc_info`` - a `ProcInfoHandler object <https://github.com/ros2/launch/blob/master/launch_testing/launch_testing/proc_info_handler.py>`__
* ``self.proc_output`` - an `IoHandler object <https://github.com/ros2/launch/blob/master/launch_testing/launch_testing/io_handler.py>`__

These objects provide dictionary like access to information about the running processes. They also contain methods that the active tests can use to wait for a process to exit or to wait for specific output.

Further reading
---------------
* `ROS2 launch system design document <https://design.ros2.org/articles/roslaunch.html>`__
* `Architecture of launch <https://github.com/ros2/launch/blob/master/launch/doc/source/architecture.rst>`__
* `Launch testing readme <https://github.com/ros2/launch/tree/master/launch_testing#readme>`__ (GitHub repository)
