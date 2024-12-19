Ament Lint CLI Utilities
========================

**Goal:** Learn how to use ``ament_lint`` and related tools to identify and fix code quality issues.

**Tutorial level:** Advanced

**Time:** 10 minutes

.. contents:: Table of Contents
   :depth: 2
   :local:

Background
----------

The ``ament`` family of CLI tools are Python tools used for software development with ROS 2.
Ament tools can be used from any build system, but a subset of these tools, the ``ament_cmake`` tools, are designed specifically to CMake-based development easier.
Ament ships with a collection of CLI programs that can help users write code that meet the ROS 2 coding standards.
Using these tools can greatly increase development velocity and help users write ROS applications and core code that meet `the ROS project's coding standards <../../The-ROS2-Project/Contributing/Code-Style-Language-Versions>`.
We recommend that ROS developers familiarize themselves with these tools and use them before submitting their pull requests.

Prerequisites
-------------

You should have the ``ament`` packages installed as part of your regular ROS 2 setup.

If you need to install ROS 2, see the :doc:`Installation instructions <../../Installation>`.


Ament Lint CLI Tools
--------------------

All ament linting tools use a similar CLI pattern.
They take in a directory, a list of directories, file, or list of files, analyze the input files, and generate a report.
All ament linting tools have the following built-in options.
**The most up to date and accurate documentation for a given ament tool can be found by using the tools built in ``--help`` functionality.**

* ``-h, --help`` - shows a help message and exit.
  The built-in help messages usually have the most accurate and up-to-date documentation of the tool.
* ``--exclude [filename ...]`` - The filenames to exclude from analysis, including wildcards.
* ``--xunit-file XUNIT_FILE`` - Generate a `xunit <https://xunit.net/>`_ compliant XML file.
  These files are most commonly used by IDEs and CI to automate the ingestion of test results.



1 ``ament_copyright``
^^^^^^^^^^^^^^^^^^^^^

The ``ament_copyright`` CLI can be used to check and update the copyright declaration in ROS source code.
This tool can also be used to check for the presence of an appropriate software license, copyright year, and copyright holders in your source code.
The ``ament_copyright`` tool works relative to the directory in which it is called, and walks the subdirectories and checks each source file within the directory.
You can use ``ament_copyright`` to check your ROS package, ROS workspace, directory, or a single source file by simply moving to the appropriate root directory and calling the command.
``ament_copyright`` can also be used to used to automatically apply a copyright and license to source code files that are missing them.


1.1 ``ament_copyright`` Arguments
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

By default ``ament_copyright`` walks the directory in which it is called, including subdirectories and returns a report that lists all files that are missing a copyright notice.
The program takes a single optional argument which is a list of directories that should be scanned for the report.
For example, if you wish to scan just source and header files for copyright notices you can call: ``ament_copyright ./src ./include``.

1.2 ``ament_copyright`` Options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``ament_copyright`` supports the following options:

* ``--add-missing COPYRIGHT_NAME LICENSE`` - Add missing copyright notice and license information using the passed copyright holder and license. ``LICENSE`` passed to this option is the name of the license to be used. A full list of available licenses can be found by calling ``ament_copyright --list-licenses``
* ``--add-copyright-year`` - Add the current year to existing copyright notices.
* ``--list-copyright-names`` - List names of known copyright holders.
* ``--list-licenses`` - List names of known licenses.
* ``--verbose`` - Show all files instead of only the ones with errors / modifications.

1.3 ``ament_copyright`` Example
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To check if your ROS package has an appropriate copyright and license file simply call ``ament_copyright`` with no arguments.
Using the ``--verbose`` option will list all checked files.

.. code-block:: console

  ament_copyright --verbose
  my_package/src/new_file.cpp: could not find copyright notice
  my_package/src/old_file.cpp: copyright=Open Source Robotics Foundation, Inc. (2023), license=apache2
  my_package/include/new_file.h: could not find copyright notice
  my_package/include/old_file.h: copyright=Open Source Robotics Foundation, Inc. (2023), license=apache2


2 ``ament_cppcheck``
^^^^^^^^^^^^^^^^^^^^

The ``ament_cppcheck`` command line tool can be used to perform static analysis of C++ source code files.
`Static analysis <https://en.wikipedia.org/wiki/Static_program_analysis>`_ is the process of automatically reviewing source code files for patterns that can often cause issues after compilation.
Some versions of `cppcheck <https://github.com/danmar/cppcheck>`__, the underlying utility used by ``ament_cppcheck``, can be rather slow.
For this reason ``ament_cppcheck`` may be disabled on some systems.
To enable it, you simply need to set the ``AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS`` environment variable.


2.1 ``ament_cppcheck`` Arguments
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

By default ``ament_cppcheck`` walks the directory in which it is called, including subdirectories and returns a report that lists all of the potential issues in a source code file.
The program takes a single optional argument which is a list of directories that should be scanned for the report.
For example, if you wish to scan just a recently modified file you can call ``ament_cppcheck ./src/my_cpp_file.cpp``.

2.2 ``ament_cppcheck`` Options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``ament_cppcheck`` supports the following options:

* ``--libraries [LIBRARIES ...]`` - Library configurations to load in addition to the standard libraries of C and C++. Each library is passed to cppcheck as '--library=<library_name>'
* ``--include_dirs [INCLUDE_DIRS ...]`` - Include directories for C/C++ files being checked.Each directory is passed to cppcheck as '-I <include_dir>' (default: None)
* ``--cppcheck-version`` - Get the cppcheck version, print it, and then exit.

2.3 ``ament_cppcheck`` Example
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Create the following simple C++ program in a file named ``example.cpp``.

.. code-block:: cpp

  int main()
  {
      char a[10];
      a[10] = 0;
      return 0;
  }


This simple program accesses a part of memory out of bounds of the allocated array.
Running ``ament_cppcheck`` in the directory with the file will yield the following results:

.. code-block:: console

   > ament_cppcheck
   [example.cpp:4]: (error: arrayIndexOutOfBounds) Array 'a[10]' accessed at index 10, which is out of bounds.
   >


3 ``ament_cpplint``
^^^^^^^^^^^^^^^^^^^

``ament_cpplint`` can be used to check your C++ code against the `Google style conventions <https://google.github.io/styleguide/cppguide.html>`_ using `cpplint <https://github.com/cpplint/cpplint?tab=readme-ov-file>`_.
``ament_cpplint`` will scan the current directory and subdirectories for all C++ header and source files and apply the CppLint application to the file and return the results.
At this time ``ament_cpplint`` is unable to address issues it finds automatically, if you would like to fix the formatting issues automatically please see ``ament_uncrustify``.


3.1 ``ament_cpplint`` Arguments
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The program takes a single optional argument which is a list of directories that should be scanned for the report.
For example, if you wish to scan just source and header files for copyright notices you can call: ``ament_copyright ./src ./include``.


3.2 ``ament_cpplint`` Options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* ``--filters FILTER,FILTER,...`` - A comma separated list of category filters to apply.
* ``--linelength N`` - The maximum line length (default: 100).
* ``--root ROOT`` - The --root option for cpplint.


3.3 ``ament_cpplint`` Example
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Let's create a simple C++ program named ``example.cpp``.
We will add a few lines of code that violate coding standards:

.. code-block:: cpp

  int main()
  {
    int a = 10;
    int b = 10;
    int c = 0;/*<trailing whitespace>*/
    if( a == b)  {/*<tab>*/      c=a;}/*<trailing whitespace>*/
    return 0;
  }


Applying ``ament_cpplint`` to this file will yield the following errors:

.. code-block:: console

  example.cpp:0:  No copyright message found.  You should have a line: "Copyright [year] <Copyright Owner>"  [legal/copyright] [5]
  example.cpp:6:  Line ends in whitespace.  Consider deleting these extra spaces.  [whitespace/end_of_line] [4]
  example.cpp:6:  Tab found; better to use spaces  [whitespace/tab] [1]
  example.cpp:6:  Line ends in whitespace.  Consider deleting these extra spaces.  [whitespace/end_of_line] [4]
  example.cpp:6:  Missing spaces around =  [whitespace/operators] [4]


4 ``ament_flake8``
^^^^^^^^^^^^^^^^^^

`Flake8 <https://pypi.org/project/flake8/>`_ is a Python tool for linting and style enforcement.
The ``ament_flake8`` command line tool can be used to quickly perform linting of Python source code files using `Flake8 <https://pypi.org/project/flake8/>`_.
This tool will help you locate minor errors and style problems with your ROS Python programs such as trailing whitespace, overly long lines of code, poorly spaced function arguments, and much more!
Note, however, that ``flake8`` and ``ament_flake8`` cannot automatically reformat code to fix these issues.

4.1 ``ament_flake8`` Arguments
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The program takes a single optional argument which is a list of directories that should be scanned for the report.
For example, if you wish to scan just one package in your workspace you can call ``ament_flake8`` directly in the package's working directory or pass it a path to the directory.


4.2 ``ament_flake8`` Options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* ``--config path`` - The config file used.
  The default config file can be found in your installation's site packages directory.
  We do not recommend changing the default settings.
* ``--linelength N`` - Manually set the maximum line length.

4.3 ``ament_flake8`` Example
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Create the following simple Python program in a file named ``example.py``.

.. code-block:: python

  def uglyPythonFunction(a,b,  c):
      if a != b:
          print("A does not match b")
      thisIsAVariableNameThatIsWayTooLongLongLong = 2
      extra_long =(thisIsAVariableNameThatIsWayTooLongLongLong*thisIsAVariableNameThatIsWayTooLongLongLong )
      return(c)

Applying ``ament_flake8`` to this file will result in the following errors.

.. code-block:: console

  example.py:1:25: E231 missing whitespace after ','
  def uglyPythonFunction(a,b,  c):

  example.py:5:5: F841 local variable 'extra_long' is assigned to but never used
      extra_long =(thisIsAVariableNameThatIsWayTooLongLongLong*thisIsAVariableNameThatIsWayTooLongLongLong )
      ^

  example.py:5:17: E225 missing whitespace around operator
      extra_long =(thisIsAVariableNameThatIsWayTooLongLongLong*thisIsAVariableNameThatIsWayTooLongLongLong )
                  ^

  example.py:5:100: E501 line too long (106 > 99 characters)
      extra_long =(thisIsAVariableNameThatIsWayTooLongLongLong*thisIsAVariableNameThatIsWayTooLongLongLong )
                                                                                                     ^

  example.py:5:105: E202 whitespace before ')'
      extra_long =(thisIsAVariableNameThatIsWayTooLongLongLong*thisIsAVariableNameThatIsWayTooLongLongLong )
                                                                                                          ^

  1     E202 whitespace before ')'
  1     E225 missing whitespace around operator
  1     E231 missing whitespace after ','
  1     E501 line too long (106 > 99 characters)
  1     F841 local variable 'extra_long' is assigned to but never used

  1 files checked
  5 errors

  'E'-type errors: 4
  'F'-type errors: 1

  Checked files:

  * example.py


5 ``ament_uncrustify``
^^^^^^^^^^^^^^^^^^^^^^

`Uncrustify <https://github.com/uncrustify/uncrustify>`_ is a C++ linting tool, similar to ``ament_cpplint``, that has the advantage that it can **automatically fix** the issues it finds!
This tool will help you locate and fix minor errors and style problems with your C++ ROS programs such as trailing whitespace, overly long lines of code, poorly spaced function arguments, and much more!


5.1 ``ament_uncrustify`` Arguments
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The program takes a single optional argument which is a list of directories that should be scanned for the report.
For example, if you wish to scan just one package in your workspace you can call ``ament_uncrustify`` directly in the package's working directory or pass it a path to the directory.


5.2 ``ament_uncrustify`` Options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* ``-c CFG`` - The config file that Uncrustify should use if you would prefer to use your own settings. We recommend you stick to the defaults
* ``--linelength N`` - The maximum line length.
* ``--language`` - One of {C,C++,CPP}, passed to uncrustify as '-l <language>' to force a specific language rather then choosing one based on file extension.
* ``--reformat`` -  Reformat the files in place, i.e. fix the formatting errors encountered. **We recommend you use this option when running ``ament_uncrustify`` as it will save you quite a bit of time!**

5.3 ``ament_uncrustify`` Example
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Let's return to the simple C++ program named ``example.cpp``.

.. code-block:: cpp

  int main()
  {
       int a = 10;
       int b = 10;
       int c = 0;<trailing whitespace>
       if( a == b)<trailing whitespace>{
    <tab>      c=a;}<trailing whitespace>
       return 0;
   }


Applying ``ament_uncrustify example.cpp`` to this file will yield the following output.

.. code-block:: diff

  --- example.cpp
  +++ example.cpp.uncrustify
  @@ -1,9 +1,10 @@
  -  int main()
  -  {
  -       int a = 10;
  -       int b = 10;
  -       int c = 0;<trailing whitespace>
  -       if( a == b)<trailing whitespace>{
  - <tab>       c=a;}<trailing whitespace>
  -       return 0;
  -   }
  +int main()
  +{
  +  int a = 10;
  +  int b = 10;
  +  int c = 0;
  +  if (a == b) {
  +    c = a;
  +  }
  +  return 0;
  +}
  1 files with code style divergence

To apply these changes to the file we can run ``ament_uncrustify`` with the ``--reformat`` flag.
**With this option specified uncrustify will apply the necessary changes in place, saving us a lot of time, especially when working with a larger codebase!**

6 Other Ament Tools Of Note
^^^^^^^^^^^^^^^^^^^^^^^^^^^

ROS Desktop Full ships with a handful of ament development tools that are worth noting.
A few of these tools are listed below.

* ``ament_lint_cmake`` - Check CMake files against the style conventions.
* ``ament_xmllint`` - Check XML markup, such as XML launch files, using xmllint.
* ``ament_pep257`` - Check Python docstrings against the style conventions in `PEP 257 <https://peps.python.org/pep-0257/>`_.

Ament is highly extensible and ROS users are encouraged to build and use ament tools that make them more productive.
You can search for other community contributed ament lint tools by using the ``apt search`` or by `searching for ament on ROS Index <https://index.ros.org/search/?term=ament>`_.
