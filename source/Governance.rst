.. _Governance:

Project Governance
==================

Technical Steering Committee (TSC)
----------------------------------
Since the beginning of ROS, the project has been overseen and prioritized primarily by one organization, first Willow Garage and now Open Robotics.
That approach has worked well enough, as evidenced by the widespread adoption of ROS around the world.

But with ROS 2, we want to broaden participation to accelerate ROS 2 delivery, starting with these areas: determining the roadmap, developing core tools and libraries, and establishing working groups to focus on important topics.
To that end, we've established a Technical Steering Committee (TSC).
As described in the :ref:`charter <ROS2TSCCharter>`, the TSC comprises representatives of organizations that are contributing to the development of ROS 2, and it has the responsibility to set the technical direction for the project.

Packages relevant to this ROS 2 TSC are listed in `REP 2005 <https://www.ros.org/reps/rep-2005.html>`_.

Meeting notes can be found on `ROS Discourse <https://discourse.ros.org/tag/tsc>`_.

The current members of the ROS 2 TSC are (20 as of 2021-01-21):

.. list-table::
   :align: center
   :widths: auto

   * - .. figure:: Governance/adlink.svg
          :alt: ADLINK Technology logo
          :height: 35px
          :target: https://www.adlinktech.com
     - ADLINK Technology: Joe Speed
   * - .. figure:: Governance/amazon.svg
          :alt: Amazon logo
          :height: 35px
          :target: https://www.amazon.com
     - Amazon: Aaron Blasdel
   * - .. figure:: Governance/apex.png
          :alt: Apex.AI logo
          :height: 35px
          :target: https://www.apex.ai
     - Apex.AI: Dejan Pangercic
   * - .. figure:: Governance/bosch_75h.jpg
          :alt: Bosch logo
          :height: 35px
          :target: https://www.bosch.com/
     - Bosch: Karsten Knese
   * - .. figure:: Governance/ubuntu.svg
          :alt: Ubuntu logo
          :height: 35px
          :target: https://ubuntu.com/
     - Canonical: Kyle Fazzari
   * - .. figure:: Governance/eprosima.svg
          :alt: eProsima logo
          :height: 35px
          :target: https://eprosima.com/
     - eProsima: Jaime Martin Losa
   * - .. figure:: Governance/gvsc.png
          :alt: GVSC logo
          :height: 50px
          :target: https://gvsc.army.mil/
     - GVSC: Jerry Towler (SwRI)
   * - .. figure:: Governance/intel.svg
          :alt: Intel logo
          :height: 50px
          :target: https://www.intel.com
     - Intel: Harold Yang
   * - .. figure:: Governance/irobot.png
          :alt: iRobot logo
          :height: 35px
          :target: https://www.irobot.com
     - iRobot: Ori Taka
   * - .. figure:: Governance/lge.svg
          :alt: LG Electronics logo
          :height: 35px
          :target: https://www.lg.com/
     - LG Electronics: Lokesh Kumar Goel
   * - .. figure:: Governance/microsoft.svg
          :alt: Microsoft logo
          :height: 35px
          :target: https://www.microsoft.com
     - Microsoft: Sean Yen
   * - .. figure:: Governance/openrobotics-logo-stacked.png
          :alt: Open Robotics logo
          :height: 35px
          :target: https://www.openrobotics.org
     - Open Robotics: Chris Lalancette
   * - .. figure:: Governance/picknik.jpg
          :alt: PickNik logo
          :height: 35px
          :target: https://picknik.ai/
     - PickNik: Dave Coleman
   * - .. figure:: Governance/robotis.png
          :alt: ROBOTIS logo
          :height: 35px
          :target: https://www.robotis.com/
     - ROBOTIS: Will Son
   * - .. figure:: Governance/rover.png
          :alt: Rover Robotics logo
          :height: 35px
          :target: https://roverrobotics.com/
     - Rover Robotics: Nick Fragale
   * - .. figure:: Governance/samsung.svg
          :alt: Samsung logo
          :height: 25px
          :target: https://www.samsung.com
     - Samsung: Steven Macenski
   * - .. figure:: Governance/sony.png
          :alt: Sony logo
          :height: 60px
          :target: https://www.sony.com
     - Sony: Tomoya Fujita
   * - .. figure:: Governance/TierIV.png
          :alt: Tier IV logo
          :height: 50px
          :target: https://www.tier4.jp/
     - Tier IV: Jilada Eccleston
   * - .. figure:: Governance/tri_logo_landscape-web.svg
          :alt: TRI logo
          :height: 50px
          :target: https://www.tri.global/
     - Toyota Research Institute: Ian McMahon
   * - .. figure:: Governance/windriver.png
          :alt: Wind River logo
          :height: 60px
          :target: https://www.windriver.com/
     - Wind River: `Andrei Kholodnyi <https://github.com/razr>`__

If you are interested in joining the ROS 2 TSC, please inquire via info@openrobotics.org.

.. toctree::
   :maxdepth: 1

   Governance/ROS2-TSC-Charter

Working Groups (WGs)
--------------------

As described in its :ref:`charter <ROS2TSCCharter>`, the TSC establishes working groups (WGs) to discuss and make progress on specific topics.

The current WGs are (11 as of 2021-01-12):

Control
^^^^^^^

* Lead(s): Bence Magyar, Karsten Knese
* Resources:

 * Meeting invite group `ros-control-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-control-working-group-invites>`_
 * Discourse tag: `wg-ros2-control <https://discourse.ros.org/tags/wg-ros2-control>`_

Edge AI
^^^^^^^

* Lead(s): Joe Speed
* Resources:

 * Meeting invite group `ros-edge-ai-working-group-invites <https://groups.google.com/forum/#!forum/ros-edge-ai-working-group-invites>`_
 * Discourse tag: `wg-edgeai <https://discourse.ros.org/tag/wg-edgeai>`_

Embedded Systems
^^^^^^^^^^^^^^^^

* Lead(s): Francesca Finocchiaro
* Resources:

 * `2019-07-29 meeting notes <https://discourse.ros.org/uploads/short-url/z1caIm7m5IVP4cPJUwg3Chq36wO.pdf>`__
 * `2019-01-15 meeting notes <https://discourse.ros.org/t/ros2-embedded-sig-meeting-2/7243/5>`__
 * Meeting invite group `ros-embedded-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-embedded-working-group-invites>`_
 * Discourse tag: `wg-embedded <https://discourse.ros.org/tags/wg-embedded>`_

Middleware
^^^^^^^^^^

* Lead(s): William Woodall
* Resources:

 * Meeting invite group `ros-middleware-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-middleware-working-group-invites>`_
 * Discourse tag: `wg-middleware <https://discourse.ros.org/tags/wg-middleware>`_

Navigation
^^^^^^^^^^

* Lead(s): Steve Macenski
* Resources:

 * `2019-03-17 meeting notes <https://discourse.ros.org/t/ros2-navigation-wg-thursday-3-00-pm-pacific-gmt-7-00/7586/9>`__

 * Meeting invite group `ros-navigation-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-navigation-working-group-invites>`_
 * Discourse tag: `wg-navigation <https://discourse.ros.org/tags/wg-navigation>`_
 * Discourse Channel: `Navigation Stack <https://discourse.ros.org/c/navigation/44>`_

Manipulation
^^^^^^^^^^^^

* Lead(s): Dave Coleman, Mark Moll
* Resources:

 * `About our working group meetings <https://discourse.ros.org/t/moveit-maintainer-meeting-all-invited-july-25th/9899>`__

 * Meeting invite group `ros-manipulation-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-manipulation-working-group-invites>`_
 * Discourse tag: `wg-moveit <https://discourse.ros.org/tags/wg-moveit>`_
 * Discourse Channel: `MoveIt <https://discourse.ros.org/c/moveit>`_

Real-time
^^^^^^^^^

* Lead(s): Dejan Pangercic, Andrei Kholodnyi
* Resources: TODO

 * `ROS 2 Real-time Working Group Community <https://github.com/ros-realtime/community>`__
 * Meeting invite group `ros-real-time-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-real-time-working-group-invites>`_
 * Discourse tag: `wg-real-time <https://discourse.ros.org/tags/wg-real-time>`_

Rust
^^^^

* Lead(s): Ruffin White, Geoffrey Biggs
* Resources:

 * `Working group Community <https://github.com/ros2-rust/rust-wg>`__
 * Meeting invite group `ros-rust-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-rust-working-group-invites>`_
 * Discourse tag: `wg-rust <https://discourse.ros.org/tags/wg-rust>`_
 * Matrix chat `+rosorg-rust:matrix.org <https://matrix.to/#/+rosorg-rust:matrix.org>`_

Safety
^^^^^^

* Lead(s): Geoffrey Biggs
* Resources:

 * `Working group website <http://ros-safety.github.io/safety_working_group/>`__
 * `Working group Community <https://github.com/ros-safety/safety_working_group>`__
 * Meeting invite group `ros-safety-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-safety-working-group-invites>`_
 * Discourse tag: `wg-safety-critical <https://discourse.ros.org/tags/wg-safety-critical>`_

Security
^^^^^^^^

* Lead(s): Sid Faber, Kyle Fazzari
* Resources:

 * `ROS 2 Security Working Group Community <https://github.com/ros-security/community>`__
 * Meeting invite group `ros-security-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-security-working-group-invites>`_
 * Discourse tag: `wg-security <https://discourse.ros.org/tags/wg-security>`_
 * Matrix chat `+rosorg-security:matrix.org <https://matrix.to/#/+rosorg-security:matrix.org>`_

Tooling
^^^^^^^

* Lead(s): Emerson Knapp, Thomas Moulard
* Resources:

 * `ROS 2 Tooling Working Group Community <https://github.com/ros-tooling/community>`__
 * Meeting invite group `ros-tooling-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-tooling-working-group-invites>`_
 * Discourse tag: `wg-tooling <https://discourse.ros.org/tags/wg-tooling>`_
 * Matrix chat `+ros-tooling:matrix.org <https://matrix.to/#/+ros-tooling:matrix.org>`_


If you'd like to join an existing ROS 2 WG, please contact the appropriate group lead(s) directly.
If you'd like to create a new WG, please inquire via info@openrobotics.org.


Working Group Policies

 * Meetings should be posted to the google calendar as well as announced on Discourse.
 * Meetings should have notes and be posted to Discourse using appropriate working group tag.
 * For attending the groups meetings please join the associated google group to get invites automatically.

Upcoming ROS Events
-------------------

Upcoming Working group meetings can be found in this `Google Calendar <https://calendar.google.com/calendar/embed?src=agf3kajirket8khktupm9go748%40group.calendar.google.com&ctz=America%2FLos_Angeles>`_.
It can be accessed via `iCal <https://calendar.google.com/calendar/ical/agf3kajirket8khktupm9go748%40group.calendar.google.com/public/basic.ics>`_.

.. raw:: html

    <iframe src="https://calendar.google.com/calendar/embed?src=agf3kajirket8khktupm9go748%40group.calendar.google.com" style="border: 0" width="800" height="600" frameborder="0" scrolling="no"></iframe>



If you have an individual event or series of events that you'd like to post please contact info@openrobotics.org
