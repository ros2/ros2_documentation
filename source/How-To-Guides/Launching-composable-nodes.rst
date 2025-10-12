Using ROS 2 launch to launch composable nodes
=============================================

.. contents:: Table of Contents
   :depth: 1
   :local:

In the :doc:`Composition tutorial <../Tutorials/Intermediate/Composition>`, you learned about composable nodes and how to use them from the command-line.
In the :doc:`Launch tutorials <../Tutorials/Intermediate/Launch/Launch-Main>`, you learned about launch files and how to use them to manage multiple nodes.

This guide will combine the above two topics and teach you how to write launch files for composable nodes.

Setup
-----

See the :doc:`installation instructions <../Installation>` for details on installing ROS 2.

If you've installed ROS 2 from packages, ensure that you have ``ros-{DISTRO}-image-tools`` installed.
If you downloaded the archive or built ROS 2 from source, it will already be part of the installation.

Launch file examples
--------------------

Below is a launch file that launches composable nodes in XML, YAML, and Python.
The launch files all do the following:

* Instantiate a cam2image composable node with remappings, custom parameters, and extra arguments
* Instantiate a showimage composable node with remappings, custom parameters, and extra arguments

.. tabs::

  .. group-tab:: XML

    .. literalinclude:: launch/composition_launch.xml
      :language: xml

  .. group-tab:: YAML

    .. literalinclude:: launch/composition_launch.yaml
      :language: yaml

  .. group-tab:: Python

    .. literalinclude:: launch/composition_launch.py
      :language: python


Loading composable nodes into an existing container
---------------------------------------------------

Containers can sometimes be launched by other launch files or from a commandline.
In that case, you need to add your components to an existing container.
For this, you may use ``LoadComposableNodes`` to load components into a given container.
The below example launches the same nodes as above.

.. tabs::

  .. group-tab:: XML

    .. literalinclude:: launch/composition_load_launch.xml
      :language: xml

  .. group-tab:: YAML

    .. literalinclude:: launch/composition_load_launch.yaml
      :language: yaml

  .. group-tab:: Python

    .. literalinclude:: launch/composition_load_launch.py
      :language: python


Using the Launch files from the command-line
--------------------------------------------

Any of the launch files above can be run with ``ros2 launch``.
Copy the data into a local file, and then run:

.. code-block:: console

  $ ros2 launch <path_to_launch_file>

Intra-process communications
----------------------------

All of the above examples use an extra argument to setup intra-process communication between the nodes.
For more information on what intra-process communications are, see the :doc:`intra-process comms tutorial <../Tutorials/Demos/Intra-Process-Communication>`.

XML, YAML, or Python: Which should I use?
-----------------------------------------

See the :ref:`discussion <launch-file-different-formats-which>` in :doc:`Launch-file-different-formats` for more information.
