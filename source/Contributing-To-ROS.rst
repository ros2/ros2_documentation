Contributing to ROS
===================

ROS (Robot Operating System) is an open-source ecosystem. We rely on contributions from our community to help us grow and improve. 
This article introduces the ways in which you can contribute to ROS.

**Area: community, contributing | Content-type: about | Experience: beginner, intermediate, expert**

.. contents:: Table of Contents
   :local:

Summary
-------   

ROS is an open source set of tools and libraries that depends on contributions from our community. Whether you are a developer, researcher, company, or student, your contribution helps us to grow and improve.

Your first contribution
-----------------------
Everyone can contribute to ROS. Depending on your skills and experience, you can review and update the ROS documentation, contribute code, or fix issues.

**Complete beginner?**

* Work through the beginner tutorials, checking them for accuracy: :ref:`test-and-review-documentation`

**Getting familiar with ROS?**

* Check the issue lists and fix one of the issues labelled "good first issue" (QUESTION FOR THE REVIEWER: Are you still using the "good first issues" label?): :ref:`contribute-to-code`

OR

* Triage an issue: :ref:`triage-an-issue`

**Experienced robotics developer?**

* Follow the discussion on Stack Exchange, and offer support: :ref:`provide-support` (QUESTION FOR THE REVIEWER: This is a placeholder suggestion. Is there something better we should encourage experienced developers to take on as their first contribution?) 

Contribute to ROS development
-----------------------------

Anyone can contribute to the development of ROS. If you are a complete beginner looking for an entry point, start with a simple contribution such as fixing a small issue. If you are a more experienced developer, consider contributing at a deeper level by designing new features, reviewing code from other developers, or fixing complex issues.

.. _contribute-to-code:

Contribute to code
~~~~~~~~~~~~~~~~~~

Contributing to our open source involves reviewing an existing project and making improvements such as fixing issues or developing features.

**Ready to contribute?**

1. Familiarize yourself with our developer guidelines and tools: `Developer Guide <https://docs.ros.org/en/lyrical/The-ROS2-Project/Contributing/Developer-Guide.html>`_

2. Pick an issue from one of the issue lists: 
   
   * `Good first issues <https://github.com/search?q=user%3Aament+user%3Aros2+is%3Aopen+label%3A%22good+first+issue%22&type=Issues>`_
   
   * `Help wanted issues <https://github.com/search?q=user%3Aament+user%3Aros2+is%3Aopen+label%3A%22help+wanted%22&type=Issues>`_ 
  
3. Fix the issue and submit the pull request <Placeholder link to new "Make a pull request" article>

Review a pull request
~~~~~~~~~~~~~~~~~~~~~

Reviewing a pull request involves carefully checking proposed changes to make sure they are correct, well written, and do not introduce issues. Anyone can review a pull request. If you are a beginner, reviewing a pull request is a great way to learn the codebase and ask questions where something is unclear. If you are a more experienced developer, you can contribute by identifying performance or security issues and ensuring overall code quality.

**Ready to contribute?**
  
* Review your first pull request: <Placeholder link to new "Reviewing a pull request" article>

Contribute to QA
~~~~~~~~~~~~~~~~~

Report an issue
^^^^^^^^^^^^^^^

Reporting issues involves identifying and describing problems in the software so they can be understood and fixed. If you are a beginner, you can report issues you encounter, for example, when following a tutorial, and include the steps needed to reproduce the problem. If you are a more experienced contributor, you can provide detailed technical information to help developers quickly diagnose and resolve the issue.

**Ready to contribute?**

* Report your first issue: <Placeholder link to new "Reporting an issue" article> 

.. _triage-an-issue:
Triage an issue
^^^^^^^^^^^^^^^

Triaging an issue involves reviewing reported issues to understand the problem, assess its impact, and decide what action to take next. If you are a beginner, you can help by checking whether an issue can be reproduced and providing additional details where needed. If you are a more experienced contributor, you can prioritize issues, identify root causes, and guide how and when they should be resolved.

**Ready to contribute?**

1. Pick an issue to triage:
  
   * `General ROS issue tracker <https://github.com/ros2/ros2/issues>`_
   
   * Other important issue trackers (QUESTION FOR THE REVIEWER: Are there any other issue trackers you suggest we link to?)

2. Triage the issue: <Placeholder link to new "Triaging an issue" article>

Report security vulnerabilities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Reporting security vulnerabilities involves responsibly identifying and disclosing potential security issues so they can be safely addressed and resolved. 

If you spot a security vulnerability, contact ros@osrfoundation.org

Contribute to future design
~~~~~~~~~~~~~~~~~~~~~~~~~~~
Contributing to design involves shaping how features are structured and interact, ensuring they are intuitive, scalable, and aligned with user needs.

Join a discussion
^^^^^^^^^^^^^^^^^

Taking part in discussions involves contributing ideas and feedback about how features and systems can be designed and improved. Every contribution is valuable, even small questions or suggestions. If you are a more experienced contributor, you can help shape design decisions, evaluate different approaches, and guide the overall direction of the project.

**Ready to contribute?**

* Join an ongoing discussion, and share your thoughts: `Open Robotics Discourse <https://discourse.openrobotics.org/c/ros/ros-general/8 />`_ (QUESTION FOR THE REVIEWER: Is this the best place to link to?)

Suggest a feature 
^^^^^^^^^^^^^^^^^

ROS continues to evolve, and suggesting features is a valuable way to help improve the experience for everyone. If you spot ways ROS can be improved, you can add your enhancement suggestion to the relevant issue tracker for others to review and support. Don't forget to check the existing list of requests first!

**Ready to contribute?**

1. Check the enhancements requests list:

   * `Feature ideas <https://docs.ros.org/en/kilted/The-ROS2-Project/Feature-Ideas.html>`_ (QUESTION FOR the REVIEWER: Is this list being maintained?)

   *  Enhancements logged in the issue trackers: For example, take a look at this `list of enhancement requests <https://github.com/ros2/ros2/issues?q=state%3Aopen%20label%3Aenhancement>`_

2. Chat to the ROS community about your suggested enhancement: `ROS Open discussion board <https://discourse.openrobotics.org/c/ros/111>`_

3. Submit an enhancement suggestion: <Placeholder link to new "Reporting an issue" article>

Improve graphical interfaces
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

ROS includes a number of graphical interfaces. Anyone can suggest interface improvements, especially those with UX design experience. This helps to shape a more intuitive and user-friendly design.

**Ready to contribute?**

* Submit a UI improvement, for example, your suggestion or your design work: <Placeholder link to new "Submitting an issue" article>

OR

* Implement a UI improvement: :ref:`contribute-to-code`


Become a core maintainer
~~~~~~~~~~~~~~~~~~~~~~~~

ROS core maintainers ensure that the project is generally making progress. The responsibilities of maintainers include:

* Reviewing incoming code contributions for style, quality, and overall fit into the goals of the repository and ROS

* Ensuring that CI continues to stay green

* Merging pull requests that meet the quality and CI standards 

* Addressing issues opened up by users

A core maintainer is responsible for guiding development, reviewing contributions, and helping ensure the quality and direction of ROS. At ROS, there are maintainer roles: as a Committer or a PMC Member. A committer can directly contribute code, while a Project Management Committee (PMC) member helps guide the overall direction and governance of the project.

**Ready to contribute?**

Each repository in the ros2 and ament organizations has a separate set of maintainers. Becoming a maintainer of one or more of those repositories is an invitation-only process, and generally involves the following steps:

* Within the last year, you have made a substantial number of code contributions to the repository.

* Within the last year, you have reviewed a substantial number incoming pull requests to the repository.

Approximately every 3 months, the ROS team will review the contributions in all of the repositories and send out invitations to new maintainers. Once the invitation is accepted, the new maintainer will be asked to go through a short training process on the mechanisms and policies of the ROS repositories. After that training process is completed, the new maintainer will be given write access to the appropriate repositories.

* Guidance for core maintainers: `Ros2 Core Maintainer Guide <https://docs.ros.org/en/kilted/How-To-Guides/Core-maintainer-guide.html>`_

Contribute to documentation
---------------------------

Contribute to improving and expanding the ROS documentation.

Write and update documentation 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Contributing to ROS documentation is a great way to learn about and improve ROS. Writing and updating documentation ensures it stays aligned with the code. Whenever you update the code, you should also the documentation so it remains accurate and reliable. Anyone can help with maintaining the existing documentation. If you are a beginner, you can help by fixing small issues such as typos or unclear instructions, while more experienced contributors can improve and expand documentation for complex features.

**Ready to contribute?**

1. Read our documentation contribution guidelines: <Placeholder link to updated contribution to docs article>

2. Browse the documentation issue list, and choose an issue to fix: `ROS documentation issue tracker <https://github.com/ros2/ros2_documentation/issues>`_

3. Update the documentation to fix the issue: <Placeholder link to new "Creating and updating documentation" article>

.. _test-and-review-documentation:
Test and review documentation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Testing and reviewing documentation makes sure it is clear, accurate, and easy to follow in practice. It involves checking that instructions work as expected and reflect how the code behaves. If you are a beginner, you can follow our tutorials and how-to guides and report anything that is unclear or incorrect. If you are a more experienced contributor, you can validate technical accuracy, identify gaps, and improve the overall structure and clarity.

**Ready to contribute?**

* Follow a tutorial, how-to guide, or other documentation. If you spot something inaccurate or unclear provide your feedback: <Placeholder link to new "Reporting an issue" article> 

.. _provide-support:
Provide support
---------------

ROS users come from a wide range of technical backgrounds and use different operating systems. Some are completely new to ROS, so even those with a little experience can play an important role in supporting others as they get started.

If you notice a question on `Robotics Stack Exchange <https://robotics.stackexchange.com/>`_ that relates to an issue you have encountered, consider sharing what worked for you. If you aren't entirely sure your answer is correct, just say so. The community is there to help, and others will step in to clarify or add more detail if needed.

**Ready to contribute?**

* Provide pointers on Robotics Stack Exchange: `Robotics Stack Exchange <https://robotics.stackexchange.com/>`_ 

* Chat with new community members by visiting the official Open Robotics Discourse: `Open Robotics Discourse <https://discourse.openrobotics.org/c/ros/111>`_

Participate in the community
----------------------------
There are various ways to join the ROS community, allowing you to participate, collaborate, and contribute effectively. Here are some ways you can join in:

* Attend events, from local informal meetings to our annual developer conference, `ROSCON <https://roscon.ros.org/OSR/>`_: 

   * Community organized events: `View the calendar <https://calendar.google.com/calendar/embed?src=c_3fc5c4d6ece9d80d49f136c1dcd54d7f44e1acefdbe87228c92ff268e85e2ea0%40group.calendar.google.com&ctz=Etc%2FUTC/>`_

   * Official Open Robotics events: `View the calendar <https://calendar.google.com/calendar/u/0/embed?src=agf3kajirket8khktupm9go748@group.calendar.google.com&ctz=Etc%2FUTC/>`_

* Host an online community group event: `Fill in the form to add your event to the calendar <https://bit.ly/OSRFCalendarForm/>`_

* Host a local meet-up: `Fill in the form to add your event to the calendar <https://bit.ly/OSRFCalendarForm/>`_

* Join discussion groups and chats:

    * `Discussions <https://discourse.openrobotics.org/c/ros/111>`_

    * `Developer chat <https://openrobotics.zulipchat.com/>`_

Promote ROS
-------------

If you want to get involved in promoting ROS, why not train or mentor others, or host a meetup in your area. 

**Ready to contribute?**

* Support people with working through the tutorials, starting with `First steps with ROS <https://docs.ros.org/en/lyrical/First-Steps.html>`_

* Be part of our community on social media: (QUESTION FOR THE REVIEWER: Can we link to Open Robotics / ROS social media channels?) <Placeholder link to ROS sm channel(s)>

Become an OSRA member
---------------------

Becoming an OSRA member lets you influence the organization's direction, collaborate with a wider community, and contribute more directly to the growth and impact of our projects.

Learn more about becoming an OSRA member https://osralliance.org/
