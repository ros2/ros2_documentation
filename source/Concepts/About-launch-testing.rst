Launch testing reference
========================

.. contents:: Contents
   :depth: 2
   :local:

Overview
--------
This documents lists the available assertions and functions in the `launch testing package <https://github.com/ros2/launch/tree/master/launch_testing>`__. 

It is recommended at the reader goes through the `launch testing guide <>`__ (TODO: Add link) first to get familiar with the general code flow and structure of launch tests before diving into this reference document. 

Assertions are generally used to check for some particular condition that establishes the result of a test case. For example, they can make sure that :

* Some given text exists in the standard streams of a process
* Processes exit with specific codes
* Some object is an instance of a specific class
* Some element exists in a given container

These are just some instances where one can use assertions and are not necessarily related to process outputs. Check out `this <https://docs.python.org/3/library/unittest.html#assert-methods>`__ link for further information.

A general template for an assertion in a test case looks like this :

::

   class TestFixture(unittest.TestCase):

      def test_1(self, proc_output):
        ...
        assert <something>

Check out this <TODO : Add link> "hello world" script for a complete example. 

*<TODO : IoHandler, ActiveIoHandler, (Active)ProcInfoHandler, usable actions reference, assertions in iohandlers >*

Assertions
----------
The package currently lists the following assertions :


* ``assertExitCodes`` : This action supports the following exit codes:

   - EXIT_OK = 0, occurs when the process exits normally.
   - EXIT_SIGINT = 130, occurs when a process was terminated by a Control-C keypress (or SIGINT).
   - EXIT_SIGQUIT = 131, occurs when a process was terminated using SIGQUIT.
   - EXIT_SIGKILL = 137, occurs when a process was terminated using SIGKILL.
   - EXIT_SIGSEGV = 139, occurs when a process exits due to a segmentation fault.

  Check out `this page <https://man7.org/linux/man-pages/man7/signal.7.html>`__ for more information on signals and the `Advanced Bash Scripting Guide <https://tldp.org/LDP/abs/html/exitcodes.html>`__ for more information on exit codes.

* ``assertInStream`` : Assert that some text was found in a stream of a process. It accepts the following arguments :

      - ``proc_output`` : The process output captured by ``launch_test``.  This is usually injected
        into test cases as ``self._proc_output``. (type: ``launch_testing.IoHandler``)
      - ``expected_output``: The output to search for. (type: ``string`` or regex pattern or a list of the aforementioned types)
      - ``process``: The process whose output will be searched. It can be of type ``string`` (search by process name) or a ``launch.actions.ExecuteProcess`` object.
      - ``cmd_args``: Optional.  If ``process`` is a string, ``cmd_args`` will be used to disambiguate
        processes with the same name.  Pass ``launch_testing.asserts.NO_CMD_ARGS`` to match a proc without
        command arguments. (argument type: ``string``)
      - ``output_filter``: Optional. A function to filter output before attempting any assertion.
      - ``strict_proc_matching``: Optional (default True), If ``proc`` is a string and the combination
         of ``proc`` and ``cmd_args`` matches multiple processes, then ``strict_proc_matching=True`` will raise
         an error. (argument type: ``bool``)
      - ``strip_ansi_escape_sequences``: If True (default), strip ansi escape
         sequences from actual output before comparing with the output filter or
         expected output. (argument type: ``bool``)
      - ``stream``: Which stream to examine.  This must be one of ``stderr`` or ``stdout``. (argument type: ``string``)

* ``assertInStderr``, ``assertInStdout`` : Assert that some text was found in the ``stdin`` and ``stdout`` streams of processes. These use ``assertInStream`` internally.

* ``assertDefaultStream`` : Return the stream that is used by default for `assertInStream`, which is typically ``stderr``

* ``assertSequentialStdout`` : Creates a context manager used to check ``stdout`` occurred in a specific order

    - ``proc_output``:  The captured output from a test run. (expected type: TODO)
    - ``process``: The process whose output will be searched. Must be a ``string`` (search by process name) or a ``launch.actions.ExecuteProcess`` object.
    - ``cmd_args``: Optional.  If ``proc`` is a string, ``cmd_args`` will be used to disambiguate
      processes with the same name.  Pass ``launch_testing.asserts.NO_CMD_ARGS`` to match a ``proc`` without
      command arguments. Must be of type ``string``.

``unittest.TestCase`` assertions
--------------------------------
Since the test fixture inherits from ``unittest.TestCase``, we can use the `assertions available for the parent class <https://docs.python.org/3/library/unittest.html#assert-methods>`__

Input/output Handler, Active input/output Handler
---------------------------------------------------


Process info handler, Active process info handler
----------------------------------------------------


Actions
-------


Further reading
---------------
* `Code for assertions <https://github.com/ros2/launch/tree/master/launch_testing/launch_testing/asserts>`__
* `Input/output Handler code <https://github.com/ros2/launch/blob/8a7649de4d65d13e24f176f2005917a9ba3061a0/launch_testing/launch_testing/io_handler.py>`__
* `Process info handler code <https://github.com/ros2/launch/blob/8a7649de4d65d13e24f176f2005917a9ba3061a0/launch_testing/launch_testing/proc_info_handler.py>`__
* `ROS specific examples <>`__ TODO: Add link
* `General examples <https://github.com/ros2/launch/tree/master/launch_testing/test/launch_testing/examples>`__ 
