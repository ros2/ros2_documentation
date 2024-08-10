Launch
======

.. contents:: Table of Contents
   :local:

Overview
--------

A ROS 2 system typically consists of multiple :doc:`nodes <About-Nodes>` running across different processes (and even different machines).
While it is possible to start each of these nodes manually, it can get cumbersome rather quickly.
Automation is also non-trivial, even if existing tools for generic process control and configuration management are leveraged, let alone crafting it from first principles (e.g. proper OS process management using shell scripts or general purpose system programming languages can be tricky on its own).

``launch`` provides the means to orchestrate the execution of ROS 2 systems, in ROS 2 terms.
_Launch files_ describe the orchestration procedure, including but not limited to which nodes to run, how to run them, and which arguments to use for them.
These descriptions are then executed, starting processes, monitoring them, reporting on their state, and even reacting to their behavior.

Moving parts
------------

``launch`` works with [_actions_](https://docs.ros.org/en/rolling/p/launch/launch.html#launch.Action), abstract representations of computational procedures with side effects on its execution environment.
Logging to standard output, executing a program, forcing a ``launch`` shutdown, are examples of actions.
An action may also encompass another action or a collection thereof to describe more complex functionality.
A collection of actions makes up the [_description_](https://docs.ros.org/en/rolling/p/launch/launch.html#launch.LaunchDescription) of a procedure that ``launch`` can take and execute.
These descriptions are typically read from [sources](https://docs.ros.org/en/rolling/p/launch/launch.html#launch.LaunchDescriptionSource), so called _launch files_, which may be written in Python, or using specific XML or YAML syntax.

While the extent of the execution environment of an action depends on its nature e.g. logging to standard output is circumscribed to the ``launch`` process whereas executing a program reaches out to the host operating system, ``launch`` maintains an execution [_context_](https://docs.ros.org/en/rolling/p/launch/launch.html#launch.LaunchContext) in which these actions take place and through which these can interact between them and with the user.
The ``launch`` context holds _configuration variables_ and propagates _events_, both of which are available to actions and to ``launch`` itself:

* Configuration variables populate the ``launch`` context as shared state.
  It is as configuration variables, for example, that arguments to ``launch`` descriptions are made available.
  Configuration variables are organized in scopes with visibility and persistence implications.
  These scopes are, however, not implicitly managed by the ``launch`` context but explicitly defined through [pushing](https://docs.ros.org/en/rolling/p/launch/launch.actions.html#launch.actions.PushLaunchConfigurations) and [popping](https://docs.ros.org/en/rolling/p/launch/launch.actions.html#launch.actions.PopLaunchConfigurations) actions.
  In general and by default, all configuration variables live in the same scope.

* [Events](https://docs.ros.org/en/rolling/p/launch/launch.html#launch.Event) are signals emitted and reacted on by actions or ``launch`` itself.
  An action completing, a process exiting, ``launch`` shutting down, are examples of events.
  These signals have no inherent side effects, only those that result from actions handling them (if any).
  Events only exist within the ``launch`` context, but are not bound to any scopes.

Actions and events are the main moving parts in ``launch``, even if events are used indirectly more often than not (i.e. it is through events that the ``launch`` internal event loop is driven).
In addition to these, and to better leverage configuration variables, ``launch`` defines _conditions_, _substitutions_, and _event handlers_:

* [Conditions](https://docs.ros.org/en/rolling/p/launch/launch.html#launch.Condition) encapsulate boolean predicates evaluated in ``launch`` context, and therefore in runtime.
  These are mainly used to define actions that execute _if_ or _unless_ a given boolean predicate turns out to be true.
* [Substitutions](https://docs.ros.org/en/rolling/p/launch/launch.html#launch.Substitution) are string interpolation expressions evaluated in ``launch`` context, though these may also tap into the larger execution environment.
  Evaluating a configuration variable value, fetching an environment variable value, retrieving the absolute path in the filesystem of an executable file, are examples of substitutions.
  Substitutions are the closest to general purpose expressions in ``launch``, enabling dynamic ``launch`` descriptions.
* [Event handlers](https://docs.ros.org/en/rolling/p/launch/launch.html#launch.EventHandler) are similar to actions, but their execution is bound to the occurrence of an specific set of events.
  These are typically defined in terms of a collection of actions to execute when and if a matching event occurs.

In a way, ``launch`` descriptions are analogous to programs in a domain specific language tailored for process orchestration, and, in particular, ROS 2 system orchestration. When composed using the building blocks available in its native Python implementation, these descriptions resemble `ASTs <https://en.wikipedia.org/wiki/Abstract_syntax_tree>`_ in procedural programming languages. The analogy has its limits, however: context is not implicitly restricted to syntactical boundaries like it would for typical variable scopes, and action execution is naturally concurrent as opposed to sequential, to name a few. However, it does bring about an important distinction that is easy to miss when writing launch files in Python: no action nor condition nor substitution carries out a computation upon instantiation but simply specifies a computation to be carried out in runtime.

.. note::

    A simple mental model to reason about Python launch files is one of two phases: a configuration phase, during which the description is fully constructed, and an execution phase, during which ``launch`` executes based on the provided description until shutdown.

Practical aspects
-----------------

* Launch files can be written in Python, XML, or YAML.
  XML and YAML launch files keep it simple and avoid the confusion that a domain specific language embedded in a general purpose programming language may bring, but the flexibility and complexity that Python launch files afford may sometimes be useful if not necessary.
  Refer to the examples in the :doc:`../../How-To-Guides/Launch-file-different-formats` guide on how to write launch files.
* Launch files can include other launch files, written in any of the supported languages, for modular system description and component reuse.
* Launch files, written in any of the supported languages, can be run using the ``ros2 launch`` command, which also can take their arguments.
  Note the difference with the ``ros2 run`` command, which works with executables installed by ROS 2 packages, not launch files.
* Most of ``launch`` infrastructure lives in the ``launch`` Python package, but ROS 2 specifics live in the ``launch_ros`` Python package.

References
----------

The most thorough reference on the design of ``launch`` is, unsurprisingly, its seminal `design document <https://design.ros2.org/articles/roslaunch.html>`__ (which even includes functionality not yet available).
[``launch`` documentation](https://docs.ros.org/en/rolling/p/launch) complements it, detailing the architecture of the core Python library.
For everything else, both ``launch`` and ``launch_ros`` APIs are documented.
