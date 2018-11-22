
Contributing to ROS 2
=====================

There are a number of ways you can contribute to the ROS 2 project.

Design discussions
------------------

Discussions about the design of ROS 2 are ongoing on the `ROS 2 forum <http://discourse.ros.org/c/ng-ros>`__.
Participating in these discussions is an important way to have a say on how different features of ROS 2 will work and will be implemented.

The diverse community behind the ROS ecosystem is one of its greatest assets.
We encourage all members of the ROS community to participate in these design discussions so that we can 1) leverage the experience of community members, and 2) keep the varied use cases of ROS in mind.

Support
-------

One of the easiest ways that you can contribute to ROS 2 is by helping other community members troubleshoot their system.
ROS 2 users come from a range of technical backgrounds, use a variety of different operating systems/platforms, and don’t necessarily have any prior experience with ROS (1 or 2).

If you see an issue on `ROS Answers <http://answers.ros.org/questions/tags:ros2/>`__ that is similar to something you’ve run into yourself, please consider providing some pointers to what helped in your situation.
+Don’t worry if you are not sure if your response is correct - simply say so, and other community members will jump in if necessary.

Contributing code
-----------------

Setting up your development environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To get started, you'll want to install from source; follow `the source installation instructions <building-from-source>` for your platform.

What to work on
^^^^^^^^^^^^^^^

We have identified a number of tasks that could be worked on by community members: they can be listed by `searching across the ROS 2 repositories for issues labeled as "help wanted" <https://github.com/search?q=user%3Aament+user%3Aros2+is%3Aopen+label%3A"help+wanted"&type=Issues>`__ or filtering the items on `the ROS 2 waffle.io board <https://waffle.io/ros2/ros2?label=help%20wanted>`__.
If you see something on that list that you would like to work on, please comment on the item to let others know that you are looking into it.

We also have a label for issues that we think should be more accessible for first-time contributors, `labelled “good first issue” <https://waffle.io/ros2/ros2?label=good%20first%20issue>`__.
If you are interested in contributing to the ROS 2 project, we encourage you to take a look at those issues first.
If you’d like to cast a wider net, we welcome contributions on any open issue (or others that you might propose), particularly tasks that have a milestone signifying they’re targeted for the next ROS 2 release (the milestone will be the next release's codename e.g. 'crystal').

If you have some code to contribute that fixes a bug or improves documentation, please submit it as a pull request to the relevant repository.
For larger changes, it is a good idea to discuss the proposal `on the ROS 2 forum <http://discourse.ros.org/c/ng-ros>`__ before you start to work on it so that you can identify if someone else is already working on something similar.
If your proposal involves changes to the APIs, it is especially recommended that you discuss the approach before starting work.

Submitting your code changes
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Code contributions should be made via pull requests to `the appropriate ros2 repositories <https://github.com/ros2>`__.

We ask all contributors to follow the practices explained in `the developer guide <Developer-Guide>`.

Please be sure to `run tests <colcon-run-the-tests>` for your code changes because most packages have tests that check that the code complies with our style guidelines.
