Launch testing reference
========================

.. contents:: Contents
   :depth: 2
   :local:

Overview
--------
This documents lists the available assertions and functions in the `launch testing package <https://github.com/ros2/launch/tree/master/launch_testing>`__. It is recommended at the reader goes through the `launch testing guide <>`__ (TODO: Add link) first to get familiar with the general code flow and structure of launch tests before diving into the reference of functions that is this document.

Assertions are used to search for some given text in the standard streams of a process and appear at the end of individual tests in a fixture. For example:

::

   class TestFixture(unittest.TestCase):

      def test_1(self, proc_output):
      ...
      assert <something>

** <TODO : IoHandler, ActiveIoHandler, (Active)ProcInfoHandler, usable actions reference, assertions in iohandlers > **

Assertions
----------
The package currently lists the following assertions :


* ``assertExitCodes`` :
  This action supports the following exit codes :
   - EXIT_OK = 0
   - EXIT_SIGINT = 130
   - EXIT_SIGQUIT = 131
   - EXIT_SIGKILL = 137
   - EXIT_SIGSEGV = 139

* ``assertInStream`` : Assert that some text was found in a stream of a process. It accepts the following arguments :
      - proc_output: The process output captured by launch_test.  This is usually injected
        into test cases as self._proc_output
      - proc_output: An launch_testing.IoHandler
      - expected_output: The output to search for
      - expected_output: string or regex pattern or a list of the aforementioned types
      - process: The process whose output will be searched
      - process: A string (search by process name) or a launch.actions.ExecuteProcess object
      -param cmd_args: Optional.  If 'process' is a string, cmd_args will be used to disambiguate
      processes with the same name.  Pass launch_testing.asserts.NO_CMD_ARGS to match a proc without
      command arguments
      - cmd_args: string
      - output_filter: Optional. A function to filter output before attempting any assertion.
      - output_filter: callable
      - strict_proc_matching: Optional (default True), If proc is a string and the combination
         of proc and cmd_args matches multiple processes, then strict_proc_matching=True will raise
         an error.
      - strict_proc_matching: bool
      - strip_ansi_escape_sequences: If True (default), strip ansi escape
         sequences from actual output before comparing with the output filter or
         expected output.
      - strip_ansi_escape_sequences: bool
      - stream: Which stream to examine.  This must be one of 'stderr' or 'stdout'.
      - stream: string

* ``assertInStderr``, ``assertInStdout`` : Assert that some text was found in the `stdin` and `stdout` streams of processes. These use `assertInStream` internally.

* ``assertDefaultStream`` : Return the stream that is used by default for `assertInStream`, which is typically `stderr`

* ``assertSequentialStdout`` : Creates a context manager used to check stdout occured in a specific order.
    - proc_output:  The captured output from a test run
    - process: The process whose output will be searched
    - process: A string (search by process name) or a launch.actions.ExecuteProcess object
    - cmd_args: Optional.  If 'proc' is a string, cmd_args will be used to disambiguate
      processes with the same name.  Pass launch_testing.asserts.NO_CMD_ARGS to match a proc without
      command arguments
    - cmd_args: string

IoHandler, ActiveIoHandler
--------------------------


ProcInfoHandler, ActiveProcInfoHandler
--------------------------------------


Actions
-------


Further reading
---------------
* `Code for assertions <https://github.com/ros2/launch/tree/master/launch_testing/launch_testing/asserts>`__
* `IoHandler code <https://github.com/ros2/launch/blob/8a7649de4d65d13e24f176f2005917a9ba3061a0/launch_testing/launch_testing/io_handler.py>`__
* `ProcInfoHandler code <https://github.com/ros2/launch/blob/8a7649de4d65d13e24f176f2005917a9ba3061a0/launch_testing/launch_testing/proc_info_handler.py>`__
* `ROS specific examples <>`__ TODO: Add link
* `General examples <https://github.com/ros2/launch/tree/master/launch_testing/test/launch_testing/examples>`__ 
