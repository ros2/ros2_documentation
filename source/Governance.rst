.. _Governance:

Project Governance
==================

Technical Steering Committee (TSC)
----------------------------------
Since the beginning of ROS, the project has been overseen and prioritized primarily by one organization, first Willow Garage and now Open Robotics.
That approach has worked well enough, as evidenced by the widespread adoption of ROS around the world.

But with ROS 2, we want to broaden participation to accelerate ROS 2 delivery, starting with these areas: determining the roadmap, developing core tools and libraries, and establishing working groups to focus on important topics.
To that end, we've established a Technical Steering Committee (TSC).
As described in the :download:`charter <Governance/ros2-tsc-charter.pdf>`, the TSC comprises representatives of organizations that are contributing to the development of ROS 2, and it has the responsibility to set the technical direction for the project.

The current members of the ROS 2 TSC are (14 as of 2019-09-20):

.. |amazon| image:: Governance/amazon.svg
   :alt: Amazon logo
   :height: 35px
   :target: https://www.amazon.com

.. |apex| image:: Governance/apex.png
   :alt: Apex.AI logo
   :height: 35px
   :target: https://www.apex.ai

.. |bosch| image:: Governance/bosch_75h.jpg
   :alt: Bosch logo
   :height: 35px
   :target: https://www.bosch.com/

.. |canonical| image:: Governance/ubuntu.svg
   :alt: Ubuntu logo
   :height: 35px
   :target: https://ubuntu.com/

.. |eprosima| image:: Governance/eprosima.svg
   :alt: eProsima logo
   :height: 35px
   :target: https://eprosima.com/

.. |gvsc| image:: Governance/gvsc.png
   :alt: GVSC logo
   :height: 35px
   :target: https://gvsc.army.mil/

.. |intel| image:: Governance/intel.svg
   :alt: Intel logo
   :height: 35px
   :target: https://www.intel.com

.. |lge| image:: Governance/lge.svg
   :alt: LG Electronics logo
   :height: 35px
   :target: https://www.lg.com/

.. |microsoft| image:: Governance/microsoft.svg
   :alt: Microsoft logo
   :height: 35px
   :target: https://www.microsoft.com

.. |openrobotics| image:: Governance/openrobotics-logo-stacked.png
   :alt: Open Robotics logo
   :height: 35px
   :target: https://www.openrobotics.org

.. |robotis| image:: Governance/robotis.png
   :alt: ROBOTIS logo
   :height: 35px
   :target: https://www.robotis.com/

.. |samsung| image:: Governance/samsung.svg
   :alt: Samsung logo
   :height: 35px
   :target: https://www.samsung.com

.. |tieriv| image:: Governance/TierIV.png
   :alt: Tier IV logo
   :height: 35px
   :target: https://www.tier4.jp/

.. |tri| image:: Governance/tri_logo_landscape-web.svg
   :alt: TRI logo
   :height: 35px
   :target: https://www.tri.global/

.. list-table::
   :align: center
   :widths: auto

   * - |amazon|
     - Amazon: Aaron Blasdel
   * - |apex|
     - Apex.AI: Dejan Pangercic
   * - |bosch|
     - Bosch: Karsten Knese
   * - |canonical|
     - Canonical: Kyle Fazzari
   * - |eprosima|
     - eProsima: Jaime Martin Losa
   * - |intel|
     - Intel: Harold Yang
   * - |lge|
     - LG Electronics: Lokesh Kumar Goel
   * - |microsoft|
     - Microsoft: Sean Yen
   * - |openrobotics|
     - Open Robotics: Dirk Thomas
   * - |robotis|
     - ROBOTIS: Yoonseok Pyo
   * - |samsung|
     - Samsung: Steven Macenski
   * - |gvsc|
     - GVSC: Jerry Towler (SwRI)
   * - |tieriv|
     - Tier IV: Geoffrey Biggs
   * - |tri|
     - Toyota Research Institute: Toffee Albina

If you are interested in joining the ROS 2 TSC, please inquire via info@openrobotics.org.

Working Groups (WGs)
--------------------

As described in its :download:`charter <Governance/ros2-tsc-charter.pdf>`, the TSC establishes working groups (WGs) to discuss and make progress on specific topics.

The current WGs are (6 as of 2019-09-04):

* Embedded Systems:

 * Lead(s): Borja Outerelo
 * Resources:

  * `2019-07-29 meeting notes <https://discourse.ros.org/uploads/short-url/z1caIm7m5IVP4cPJUwg3Chq36wO.pdf>`__
  * `2019-01-15 meeting notes <https://discourse.ros.org/t/ros2-embedded-sig-meeting-2/7243/5>`__
  * Meeting invite group `ros-embedded-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-embedded-working-group-invites>`_
  * Discourse tag: `wg-embedded <https://discourse.ros.org/tags/wg-embedded>`_

* Navigation

 * Lead(s): Steve Macenski
 * Resources:

  * `2019-03-17 meeting notes <https://discourse.ros.org/t/ros2-navigation-wg-thursday-3-00-pm-pacific-gmt-7-00/7586/9>`__

  * Meeting invite group `ros-navigation-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-navigation-working-group-invites>`_
  * Discourse tag: `wg-navigation <https://discourse.ros.org/tags/wg-navigation>`_
  * Discourse Channel: `Navigation Stack <https://discourse.ros.org/c/navigation/44>`_

* Real-time

 * Lead(s): Dejan Pangercic
 * Resources: TODO

  * Meeting invite group `ros-real-time-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-real-time-working-group-invites>`_
  * Discourse tag: `wg-real-time <https://discourse.ros.org/tags/wg-real-time>`_

* Safety

 * Lead(s): Geoffrey Biggs
 * Resources:

  * Meeting invite group `ros-safety-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-safety-working-group-invites>`_
  * Discourse tag: `wg-safety-critical <https://discourse.ros.org/tags/wg-safety-critical>`_

* Security

 * Lead(s): Joe McManus, Kyle Fazzari
 * Resources:

  * `2019-02-13 meeting notes <https://discourse.ros.org/t/ros2-security-working-group-online-meeting-feb-13th-2019-between-2-00-3-00-pm-pst/7639/2>`__
  * Meeting invite group `ros-security-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-security-working-group-invites>`_
  * Discourse tag: `wg-security <https://discourse.ros.org/tags/wg-security>`_

* Tooling

 * Lead(s): Emerson Knapp, Thomas Moulard
 * Resources:

  * Meeting invite group `ros-tooling-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-tooling-working-group-invites>`_
  * Discourse tag: `wg-tooling <https://discourse.ros.org/tags/wg-tooling>`_


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



If you have an individial event or series of events that you'd like to post please contact info@openrobotics.org
