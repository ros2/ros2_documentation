Migrating ROS 1 launchfiles
===========================

Background
----------

A description of the ROS 2 launch system and its Python API can be found in `this tutorial<Launch-system>`.
In this tutorial, will be described how to write XML launch files, which allow an easy migration from ROS 1.


Migrating tags from ROS1 to ROS2
--------------------------------

launch
^^^^^^

This tag serves the same purpose as its `ROS 1 counterpart <http://wiki.ros.org/roslaunch/XML/launch>`__.
It works as the root element of any ROS 2 launch XML file.
The deprecated attribute isn't available.

node
^^^^

As in `ROS1 <http://wiki.ros.org/roslaunch/XML/node>`__, it allows launching a new node.
The following summarize the differences:

* ``pkg`` attribute is now ``package``.
* ``type`` attribute is now executable.
* The following attributes aren't available: ``machine``, ``respawn``, ``respawn_delay``, ``clear_params``.

param
^^^^^

In ROS 2, there's no global parameters concept.
Application using global parameters should be refactored.
This tag can only be used nested in a node tag.
`ROS 1 reference <http://wiki.ros.org/roslaunch/XML/param>`__.

* There is not ``type`` attribute. See type deduction rules below.
* The following attributes aren't available: ``textfile``, ``binfile``, ``executable``, ``command``.

Type inference rules
""""""""""""""""""""

Here are some examples of how to write parameters:

.. code-block:: xml

   <node package="my_package" executable="my_executable" name="my_node">
      <!--A string parameter with value "1"-->
      <param name="a_string" value="'1'"/>
      <!--A integer parameter with value 1-->
      <param name="an_int" value="1"/>
      <!--A float parameter with value 1.0-->
      <param name="a_float" value="1.0"/>
      <!--A string parameter with value "asd"-->
      <param name="another_string" value="asd"/>
      <!--Another string parameter, with value "asd"-->
      <param name="string_with_same_value_as_above" value="'asd'"/>
      <!--Another string parameter, with value "'asd'"-->
      <param name="quoted_string" value="\'asd\'"/>
      <!--A list of strings, with value ["asd", "bsd", "csd"]-->
      <param name="list_of_strings" value="asd, bsd, csd" value-sep=", "/>
      <!--A list of ints, with value [1, 2, 3]-->
      <param name="list_of_ints" value="1,2,3" value-sep=","/>
      <!--Another list of strings, with value ["1", "2", "3"]-->
      <param name="another_list_of_strings" value="'1';'2';'3'" value-sep=";"/>
      <!--A list of strings using an strange separator, with value ["1", "2", "3"]-->
      <param name="strange_separator" value="'1'//'2'//'3'" value-sep="//"/>
   </node>

Parameter grouping
""""""""""""""""""

In ROS 2, param tags are allowed to be nested.
For example:

.. code-block:: xml

   <node package="my_package" executable="my_executable" name="my_node" ns="/an_absoulute_ns">
      <param name="group1">
         <param name="group2">
            <param name="my_param" value="1"/>
         </param>
         <param name="another_param" value="2"/>
      </param>
   </node>

That will create two parameters:
   - ``group1.group2.my_param`` of value ``1``, hosted by node ``/an_absolute_ns/my_node``.
   - ``group1.another_param`` of value ``2`` hosted by node ``/an_absolute_ns/my_node``.

rosparam
^^^^^^^^

`This tag <http://wiki.ros.org/roslaunch/XML/rosparam>`__ can be replaced using ``from`` attribute in ``param`` tag.
For example:

.. code-block:: xml

   <node package="my_package" executable="my_executable" name="my_node" ns="/an_absoulute_ns">
      <param from="/path/to/file"/>
   </node>

remap
^^^^^

Its usage is the same as in `ROS 1 <http://wiki.ros.org/roslaunch/XML/remap>`__.
The only difference, is that it can only be used nested in a ``node`` tag.

machine
^^^^^^^

There's not implementation of this feature in ROS 2 at the moment.

include
^^^^^^^

There is some difference from how it worked in ROS 1:

* In ros1, includes were scoped.
  In ROS 2, they should be nested inside a ``group`` tag for this.
* ``ns`` attribute is not supported.
  See example of ``push_ros_namespace`` tag for a workaround.
* ``arg`` tags nested in ``include`` tag doesn't support conditionals (``if`` or ``unless``).
* There is not support of ``env`` child tags. ``set_env`` and ``unset_env`` can be used as a workaround.
* ``clear_params``, ``pass_all_args`` attributes aren't supported.

arg
^^^

Similar behavior to `ROS 1 tag <http://wiki.ros.org/roslaunch/XML/arg>`__.
There are some minor changes:

* ``value`` attribute is not allowed.
  Use ``let`` tag for this.
* ``doc`` is now ``description``.
* When used nested in an include action, ``if`` and ``unless``  attributes aren't allowed.

Passing an argument via the command line
""""""""""""""""""""""""""""""""""""""""

See `ROS 2 launch tutorial <Launch-system>`__.


env
^^^

This has been replaced with ``env``, ``set_env`` and ``unset_env``.

* ``env`` can be used nested in a ``node`` or ``executable`` tag.
  It accepts the same attributes as the `ROS 1 version <http://wiki.ros.org/roslaunch/XML/env>`__, except ``if`` and ``unless`` condition.
* ``set_env`` can be used in the root tag ``launch``.
  It also accepts the same attributes, including conditionals.
* ``unset_env`` unsets an environment variable.
  It accepts a ``name`` attribute, and conditionals.

group
^^^^^

There is some differences with `ROS 1 tag <http://wiki.ros.org/roslaunch/XML/group>`__.

* There is not ``ns`` attribute.
  See the new ``push_ros_namespace`` tag as a workaround.
* ``clear_params`` attribute won't be available.
* It doesn't accept ``remap`` and ``param`` tags as children.

machine and test
^^^^^^^^^^^^^^^^

They aren't supported at the moment.

New tags
^^^^^^^^

set_env and unset_env
"""""""""""""""""""""

See ``env`` tag decription.

push_ros_namespace
""""""""""""""""""

``include`` and ``group`` tags don't accept ``ns`` attribute.
This action can be used as a workaround:

.. code-block:: xml

   <!-Other tags-->
   <group>
      <push_ros_namespace namespace="my_ns"/>
      <!--Nodes here are namespaced with "my_ns".-->
      <!--If there is an include action here, its nodes will also be namespaced.-->
      <push_ros_namespace namespace="another_ns"/>
      <!--Nodes here are namespaced with "another_ns/my_ns".-->
      <push_ros_namespace namespace="/absolute_ns"/>
      <!--Nodes here are namespaced with "/absolute_ns".-->
      <!--The following node receives an absolute namespace, so it will ignore the others previously pushed.-->
      <!--The full path of the node will be /asd/my_node.-->
      <node package="my_pkg" executable="my_executable" name="my_node" ns="/asd"/>
   </group>
   <!--Nodes outside the group action won't be namespaced.-->
   <!-Other tags-->

let
"""

It replaces ``arg`` tag with value attribute.

.. code-block:: xml

   <let var="foo" value="asd"/>

executable
""""""""""

Allows running any executable.
For example:

.. code-block:: xml

   <executable cmd="ls -las" cwd="/var/log" name="my_exec" launch-prefix="something" output="screen" shell="true">
      <env name="LD_LIBRARY" value="/lib/some.so"/>
   </executable>

Replacing include tag
^^^^^^^^^^^^^^^^^^^^^

For having exactly the same behavior as ROS 1, they should be nested in a ``group`` tag.

.. code-block:: xml

   <group>
      <include file="another_launch_file"/>
   </group>

For replacing the ``ns`` attribute usage:

.. code-block:: xml

   <group>
      <push_ros_namespace namespace="my_ns"/>
      <include file="another_launch_file"/>
   </group>

Substitutions
-------------

Substitutions syntax haven't changed, it's still ``$(sub-name val1 val2 ...)``.
There are some changes with ROS 1:

* There is not ``env`` alternative.
  ``optenv`` has been renamed as ``env``.
* ``find`` has been replaced with ``find-package``.
* There is a new ``exec_in_package`` substitution.
  e.g.: ``$(exec_in_package package_name exec_name)``
* There is a new ``find-exec`` substitution.
* ``anon`` hasn't an alternative at the moment.
* ``arg`` has been replaced with ``var``.
  It looks at configurations defined with ``arg`` or ``let`` tag.
* ``eval`` has not alternative at the moment.
* ``dirname`` has the same behaviour as before.

Type inference rules
--------------------

The rules that were shown in ``Type inference rules`` subsection of ``param`` tag applies to any attribute.
For example:

.. code-block:: xml

   <!--Setting a string value to an attribute expecting an int will raise an error.-->
   <tag1 int-attr="'1'"/>
   <!--Correct version.-->
   <tag1 int-attr="1"/>
   <!--Setting an integer in an attribute expecting a string will raise an error.-->
   <tag2 str-attr="1"/>
   <!--Correct version.-->
   <tag2 str-attr="'1'"/>
   <!--Setting a list of strings in an attribute expecting a string will raise an error.-->
   <tag3 str-attr="asd, bsd" str-attr-sep=", "/>
   <!--Correct version.-->
   <tag3 str-attr="don't use a separator"/>

Some attributes accept more than a single type, for example ``value`` attribute of ``param`` tag.
It's usual that parameters that are of type ``int`` (or ``float``) also accept an ``str``, that will be later
substituted and tried to convert to an ``int`` (or ``float``) by the action.
