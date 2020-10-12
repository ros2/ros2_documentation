.. _MigratingLaunch:

Migrating launch files from ROS 1 to ROS 2
==========================================

.. contents:: Table of Contents
   :depth: 1
   :local:

This tutorial describes how to write XML launch files for an easy migration from ROS 1.

Background
----------

A description of the ROS 2 launch system and its Python API can be found in `Launch System tutorial <Launch-system>`.


Migrating tags from ROS 1 to ROS 2
----------------------------------

launch
^^^^^^

* `Available in ROS 1 <https://wiki.ros.org/roslaunch/XML/launch>`__.
* ``launch`` is the root element of any ROS 2 launch XML file.

node
^^^^

* `Available in ROS 1 <https://wiki.ros.org/roslaunch/XML/node>`__.
* Launches a new node.
* Differences from ROS 1:
   * ``type`` attribute is now ``exec``.
   * The following attributes aren't available: ``machine``, ``respawn``, ``respawn_delay``, ``clear_params``.

Example
~~~~~~~

.. code-block:: xml

   <launch>
      <node pkg="demo_nodes_cpp" exec="talker"/>
      <node pkg="demo_nodes_cpp" exec="listener"/>
   </launch>

param
^^^^^

* `Available in ROS 1 <https://wiki.ros.org/roslaunch/XML/param>`__.
* Used for passing a parameter to a node.
* There's no global parameter concept in ROS 2.
  For that reason, it can only be used nested in a ``node`` tag.
  Some attributes aren't supported in ROS 2: ``type``, ``textfile``, ``binfile``, ``executable``, ``command``.

Example
~~~~~~~

.. code-block:: xml

   <launch>
      <node pkg="demo_nodes_cpp" exec="parameter_event">
         <param name="foo" value="5"/>
      </node>
   </launch>

Type inference rules
~~~~~~~~~~~~~~~~~~~~

Here are some examples of how to write parameters:

.. code-block:: xml

   <node pkg="my_package" exec="my_executable" name="my_node">
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
~~~~~~~~~~~~~~~~~~

In ROS 2, ``param`` tags are allowed to be nested.
For example:

.. code-block:: xml

   <node pkg="my_package" exec="my_executable" name="my_node" ns="/an_absoulute_ns">
      <param name="group1">
         <param name="group2">
            <param name="my_param" value="1"/>
         </param>
         <param name="another_param" value="2"/>
      </param>
   </node>

That will create two parameters:

* A ``group1.group2.my_param`` of value ``1``, hosted by node ``/an_absolute_ns/my_node``.
* A ``group1.another_param`` of value ``2`` hosted by node ``/an_absolute_ns/my_node``.

It's also possible to use full parameter names:

.. code-block:: xml

   <node pkg="my_package" exec="my_executable" name="my_node" ns="/an_absoulute_ns">
      <param name="group1.group2.my_param" value="1"/>
      <param name="group1.another_param" value="2"/>
   </node>

rosparam
^^^^^^^^

* `Available in ROS 1 <https://wiki.ros.org/roslaunch/XML/rosparam>`__.
* Loads parameters from a yaml file.
* It has been replaced with a ``from`` atribute in ``param`` tags.

Example
~~~~~~~

.. code-block:: xml

   <node pkg="my_package" exec="my_executable" name="my_node" ns="/an_absoulute_ns">
      <param from="/path/to/file"/>
   </node>

remap
^^^^^

* `Available in ROS 1 <https://wiki.ros.org/roslaunch/XML/remap>`__.
* Used to pass remapping rules to a node.
* It can only be used within ``node`` tags.

Example
~~~~~~~

.. code-block:: xml

   <launch>
      <node pkg="demo_nodes_cpp" exec="talker">
         <remap from="chatter" to="my_topic"/>
      </node>
      <node pkg="demo_nodes_cpp" exec="listener">
         <remap from="chatter" to="my_topic"/>
      </node>
   </launch>

include
^^^^^^^

* `Available in ROS 1 <https://wiki.ros.org/roslaunch/XML/include>`__.
* Allows including another launch file.
* Differences from ROS 1:
   * Available in ROS 1, included content was scoped.
     In ROS 2, it's not.
     Nest includes in ``group`` tags to scope them.
   * ``ns`` attribute is not supported.
     See example of ``push-ros-namespace`` tag for a workaround.
   * ``arg`` tags nested in an ``include`` tag don't support conditionals (``if`` or ``unless``).
   * There is no support for nested ``env`` tags.
     ``set_env`` and ``unset_env`` can be used instead.
   * Both ``clear_params`` and ``pass_all_args`` attributes aren't supported.

Examples
~~~~~~~~

See `Replacing an include tag`_.

arg
^^^

* `Available in ROS 1 <https://wiki.ros.org/roslaunch/XML/arg>`__.
* ``arg`` is used for declaring a launch argument, or to pass an argument when using ``include`` tags.
* Differences from ROS 1:
   * ``value`` attribute is not allowed.
     Use ``let`` tag for this.
   * ``doc`` is now ``description``.
   * When nested within an ``include`` tag, ``if`` and ``unless`` attributes aren't allowed.

Example
~~~~~~~

.. code-block:: xml

   <launch>
      <arg name="topic_name" default="chatter"/>
      <node pkg="demo_nodes_cpp" exec="talker">
         <remap from="chatter" to="$(var topic_name)"/>
      </node>
      <node pkg="demo_nodes_cpp" exec="listener">
         <remap from="chatter" to="$(var topic_name)"/>
      </node>
   </launch>

Passing an argument via the command line
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

See `ROS 2 launch tutorial <Launch-system>`.


env
^^^

* `Available in ROS 1 <https://wiki.ros.org/roslaunch/XML/env>`__.
* Sets an environment variable.
* It has been replaced with ``env``, ``set_env`` and ``unset_env``:
   * ``env`` can only be used nested in a ``node`` or ``executable`` tag.
     ``if`` and ``unless`` tags aren't supported.
   * ``set_env`` can be nested within the root tag ``launch`` or in ``group`` tags.
     It accepts the same attributes as ``env``, and also ``if`` and ``unless`` tags.
   * ``unset_env`` unsets an environment variable.
     It accepts a ``name`` attribute and conditionals.

Example
~~~~~~~

.. code-block:: xml

   <launch>
      <set_env name="MY_ENV_VAR" value="MY_VALUE" if="CONDITION_A"/>
      <set_env name="ANOTHER_ENV_VAR" value="ANOTHER_VALUE" unless="CONDITION_B"/>
      <set_env name="SOME_ENV_VAR" value="SOME_VALUE"/>
      <node pkg="MY_PACKAGE" exec="MY_EXECUTABLE" name="MY_NODE">
         <env name="NODE_ENV_VAR" value="SOME_VALUE"/>
      </node>
      <unset_env name="MY_ENV_VAR" if="CONDITION_A"/>
      <node pkg="ANOTHER_PACKAGE" exec="ANOTHER_EXECUTABLE" name="ANOTHER_NODE"/>
      <unset_env name="ANOTHER_ENV_VAR" unless="CONDITION_B"/>
      <unset_env name="SOME_ENV_VAR"/>
   </launch>


group
^^^^^

* `Available in ROS 1 <https://wiki.ros.org/roslaunch/XML/group>`__.
* Allows limiting the scope of launch configurations.
  Usually used together with ``let``, ``include`` and ``push-ros-namespace`` tags.
* Differences from ROS 1:
   * There is no ``ns`` attribute.
     See the new ``push-ros-namespace`` tag as a workaround.
   * ``clear_params`` attribute isn't available.
   * It doesn't accept ``remap`` nor ``param`` tags as children.

Example
~~~~~~~

``launch-prefix`` configuration affects both ``executable`` and ``node`` tags' actions.
This example will use ``time`` as a prefix if ``use_time_prefix_in_talker`` argument is ``1``, only for the talker.

.. code-block:: xml

   <launch>
      <arg name="use_time_prefix_in_talker" default="0"/>
      <group>
         <let name="launch-prefix" value="time" if="$(var use_time_prefix_in_talker)"/>
         <node pkg="demo_nodes_cpp" exec="talker"/>
      </group>
      <node pkg="demo_nodes_cpp" exec="listener"/>
   </launch>

machine
^^^^^^^

It is not supported at the moment.

test
^^^^

It is not supported at the moment.

New tags in ROS 2
-----------------

set_env and unset_env
^^^^^^^^^^^^^^^^^^^^^

See `env`_ tag decription.

push-ros-namespace
^^^^^^^^^^^^^^^^^^

``include`` and ``group`` tags don't accept an ``ns`` attribute.
This action can be used as a workaround:

.. code-block:: xml

   <!-Other tags-->
   <group>
      <push-ros-namespace namespace="my_ns"/>
      <!--Nodes here are namespaced with "my_ns".-->
      <!--If there is an include action here, its nodes will also be namespaced.-->
      <push-ros-namespace namespace="another_ns"/>
      <!--Nodes here are namespaced with "another_ns/my_ns".-->
      <push-ros-namespace namespace="/absolute_ns"/>
      <!--Nodes here are namespaced with "/absolute_ns".-->
      <!--The following node receives an absolute namespace, so it will ignore the others previously pushed.-->
      <!--The full path of the node will be /asd/my_node.-->
      <node pkg="my_pkg" exec="my_executable" name="my_node" ns="/asd"/>
   </group>
   <!--Nodes outside the group action won't be namespaced.-->
   <!-Other tags-->

let
^^^

It's a replacement of ``arg`` tag with a value attribute.

.. code-block:: xml

   <let var="foo" value="asd"/>

executable
^^^^^^^^^^

It allows running any executable.

Example
~~~~~~~

.. code-block:: xml

   <executable cmd="ls -las" cwd="/var/log" name="my_exec" launch-prefix="something" output="screen" shell="true">
      <env name="LD_LIBRARY" value="/lib/some.so"/>
   </executable>

Replacing an include tag
------------------------

To have exactly the same behavior as Available in ROS 1, ``include`` tags must be nested in a ``group`` tag.

.. code-block:: xml

   <group>
      <include file="another_launch_file"/>
   </group>

To replace the ``ns`` attribute, ``push-ros-namespace`` action must be used:

.. code-block:: xml

   <group>
      <push-ros-namespace namespace="my_ns"/>
      <include file="another_launch_file"/>
   </group>

Substitutions
-------------

Documentation about ROS 1's substitutions can be found in `roslaunch XML wiki <https://wiki.ros.org/roslaunch/XML>`__.
Substitutions syntax hasn't changed, i.e. it still follows the ``$(substitution-name arg1 arg2 ...)`` pattern.
There are, however, some changes w.r.t. ROS 1:

* ``env`` and ``optenv`` tags have been replaced by the ``env`` tag.
  ``$(env <NAME>)`` will fail if the environment variable doesn't exist.
  ``$(env <NAME> '')`` does the same as ROS 1's ``$(optenv <NAME>)``.
  ``$(env <NAME> <DEFAULT>)`` does the same as ROS 1's ``$(env <NAME> <DEFAULT>)`` or ``$(optenv <NAME> <DEFAULT>)``.
* ``find`` has been replaced with ``find-pkg-share`` (substituting the share directory of an installed package).
  Alternatively ``find-pkg-prefix`` will return the root of an installed package.
* There is a new ``exec-in-pkg`` substitution.
  e.g.: ``$(exec-in-pkg <package_name> <exec_name>)``.
* There is a new ``find-exec`` substitution.
* ``arg`` has been replaced with ``var``.
  It looks at configurations defined either with ``arg`` or ``let`` tag.
* ``eval`` and ``dirname`` substitutions haven't changed.
* ``anon`` substitution is not supported.

Type inference rules
--------------------

The rules that were shown in ``Type inference rules`` subsection of ``param`` tag applies to any attribute.
For example:

.. code-block:: xml

   <!--Setting a string value to an attribute expecting an int will raise an error.-->
   <tag1 attr-expecting-an-int="'1'"/>
   <!--Correct version.-->
   <tag1 attr-expecting-an-int="1"/>
   <!--Setting an integer in an attribute expecting a string will raise an error.-->
   <tag2 attr-expecting-a-str="1"/>
   <!--Correct version.-->
   <tag2 attr-expecting-a-str="'1'"/>
   <!--Setting a list of strings in an attribute expecting a string will raise an error.-->
   <tag3 attr-expecting-a-str="asd, bsd" str-attr-sep=", "/>
   <!--Correct version.-->
   <tag3 attr-expecting-a-str="don't use a separator"/>

Some attributes accept more than a single type, for example ``value`` attribute of ``param`` tag.
It's usual that parameters that are of type ``int`` (or ``float``) also accept an ``str``, that will be later substituted and tried to convert to an ``int`` (or ``float``) by the action.
