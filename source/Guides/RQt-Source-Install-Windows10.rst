.. redirect-from::

    RQt-Source-Install-Windows10

Building RQt from source on Windows 10
======================================

This page provides specific information to building RQt from source on Windows.
Follow these instructions before proceeding with the `RQt Source Install <RQt-Source-Install>` page.

If you have not done so, follow the `ROS 2 Windows Development Setup guide <../Installation/Windows-Development-Setup>` before continuing.

System Requirements
-------------------

* Windows 10
* Visual Studio 15.7.6

Currently Visual Studio 15.8 fails to build ROS 2 (`see issue <https://github.com/osrf/osrf_testing_tools_cpp/issues/15>`__).
Older versions of VS can be found `here <https://docs.microsoft.com/en-us/visualstudio/productinfo/installing-an-earlier-release-of-vs2017>`__.


Dependencies
------------

The primary dependencies of the RQt package are sip and PyQt5.
PySide2 may be supported in the future.
Even though they are provided through PyPi and chocolatey, you must install them by source to get compatible versions.

Install sip by source
^^^^^^^^^^^^^^^^^^^^^

Download from https://www.riverbankcomputing.com/software/sip/download

Run the x64 Native Tools Command Prompt as Administrator, and ``cd`` to the uncompressed source directory.

Run:

.. code-block:: bat

   python3 configure.py
   nmake
   nmake install

If ``python3`` is installed on your system as ``python``, be sure to use that program name instead.

Install PyQt5 by source
^^^^^^^^^^^^^^^^^^^^^^^

Download from https://www.riverbankcomputing.com/software/pyqt/download5

Run the x64 Native Tools Command Prompt as Administrator, and ``cd`` to the uncompressed source directory.
I ran into trouble with Qt 5.11.3 and PyQt5 compiling QtNfc, but it can be easily disabled.

.. code-block:: bat

   python3 configure.py --disable QtNfc
   nmake
   nmake install

Test that it works
^^^^^^^^^^^^^^^^^^

If install occurred without failure, try the commands below.
They should run without issue and you should see 4.19.13 as your ``sip.exe`` version.

.. code-block:: bat

   sip -V
   python3 -c "from PyQt5 import QtCore"


Other dependencies
^^^^^^^^^^^^^^^^^^

Install GraphViz from https://graphviz.gitlab.io/_pages/Download/Download_windows.html.

Install ``pydot`` and ``pyparsing``:

.. code-block:: bat

   pip3 install pydot pyparsing


PyGraphViz is a test dependency of ``qt_dotgraph``, but it is currently unsupported on Windows and building by source is not straight forward.
Manually merging this patch is the currently recommended solution (not verified):
`pygraphviz patch <https://github.com/Kagami/pygraphviz/commit/fe442dc16accb629c3feaf157af75f67ccabbd6e>`__


Install RQt by source
---------------------

Continue with the `RQt source install page <RQt-Source-Install>`.
