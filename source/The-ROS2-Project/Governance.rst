.. redirect-from::

  Governance

.. _Governance:

Project Governance
==================

.. contents:: Table of Contents
   :depth: 1
   :local:

Technical Steering Committee (TSC)
----------------------------------
Since the beginning of ROS, the project has been overseen and prioritized primarily by one organization, first Willow Garage and now Open Robotics.
That approach has worked well enough, as evidenced by the widespread adoption of ROS around the world.

But with ROS 2, we want to broaden participation to accelerate ROS 2 delivery, starting with these areas: determining the roadmap, developing core tools and libraries, and establishing working groups to focus on important topics.
To that end, we've established a Technical Steering Committee (TSC).
As described in the :doc:`charter <Governance/ROS2-TSC-Charter>`, the TSC comprises representatives of organizations that are contributing to the development of ROS 2, and it has the responsibility to set the technical direction for the project.

Packages relevant to this ROS 2 TSC are listed in `REP 2005 <https://www.ros.org/reps/rep-2005.html>`_.

Meeting notes can be found on `ROS Discourse <https://discourse.ros.org/tag/tsc>`_.

The current members of the ROS 2 TSC are (23 as of 2022-02-01):

.. |amazon| image:: Governance/images/amazon.svg
.. |apex| image:: Governance/images/apex.png
.. |bosch| image:: Governance/images/bosch_75h.jpg
.. |canonical| image:: Governance/images/ubuntu.svg
.. |eprosima| image:: Governance/images/eprosima.svg
.. |gvsc| image:: Governance/images/gvsc.png
.. |intel| image:: Governance/images/intel.svg
.. |intrinsic| image:: Governance/images/intrinsic.png
.. |irobot| image:: Governance/images/irobot.png
.. |microsoft| image:: Governance/images/microsoft.svg
.. |openrobotics| image:: Governance/images/openrobotics-logo-stacked.png
.. |picknik| image:: Governance/images/picknik.png
.. |robotis| image:: Governance/images/robotis.png
.. |ros2| image:: Governance/images/ros2_logo.png
.. |rosindustrial| image:: Governance/images/ros-industrial.png
.. |samsung| image:: Governance/images/samsung.svg
.. |sony| image:: Governance/images/sony.png
.. |tri| image:: Governance/images/tri_logo_landscape-web.svg
.. |windriver| image:: Governance/images/windriver.png
.. |foxglove| image:: Governance/images/foxglove.png
.. |zettascale| image:: Governance/images/zettascale.png
.. raw:: html

    <!--
    The CSS and HTML below generate the list of TSC members.
    It is currently using raw HTML because there was no way that I could see to make the
    ReStructured Text "list-tables" directive make the logos a consistent size.
    -->
    <style>
    table.tscclass {
      border: 1px solid #e1e4e5;
      border-top-color: rgb(225, 228, 229);
      border-top-style: solid;
      border-top-width: 1px;
      border-right-color: rgb(225, 228, 229);
      border-right-style: solid;
      border-right-width: 1px;
      border-bottom-color: rgb(225, 228, 229);
      border-bottom-style: solid;
      border-bottom-width: 1px;
      border-left-color: rgb(225, 228, 229);
      border-left-style: solid;
      border-left-width: 1px;
      border-image-outset: 0;
      border-image-repeat: stretch;
      border-image-slice: 100%;
      border-image-source: none;
      border-image-width: 1;
      width: 100%;
    }
    td.tscclass {
      border-top-color: rgb(225, 228, 229);
      border-top-style: solid;
      border-top-width: 1px;
      border-right-color: rgb(225, 228, 229);
      border-right-style: solid;
      border-right-width: 1px;
      border-bottom-color: rgb(225, 228, 229);
      border-bottom-style: solid;
      border-bottom-width: 1px;
      border-left-color: rgb(225, 228, 229);
      border-left-style: solid;
      border-left-width: 1px;
      padding-left: 15px;
    }
    tr.tscclass:nth-child(2n-1) {
      background-color: #f3f6f6;
    }
    </style>
    <table class="tscclass">
      <tbody>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><a class="reference internal" href="https://www.amazon.com"><img alt="Amazon logo" src="../_images/amazon.svg" style="height: 35px;" /></a></p></td>
          <td class="tscclass" align="center"><p>Amazon: Aaron Blasdel</p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><a class="reference internal" href="https://www.apex.ai"><img alt="Apex.AI logo" src="../_images/apex.png" style="height: 35px;" /></a></p></td>
          <td class="tscclass" align="center"><p>Apex.AI: Lyle Johnson</p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><a class="reference internal" href="https://www.bosch.com"><img alt="Bosch logo" src="../_images/bosch_75h.jpg" style="height: 35px;" /></a></p></td>
          <td class="tscclass" align="center"><p>Bosch: Christian Henkel</p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><a class="reference internal" href="https://ubuntu.com"><img alt="Ubuntu logo" src="../_images/ubuntu.svg" style="height: 35px;" /></a></p></td>
          <td class="tscclass" align="center"><p>Canonical: Jeremie Deray</p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><a class="reference internal" href="https://eprosima.com"><img alt="eProsima logo" src="../_images/eprosima.svg" style="height: 35px;" /></a></p></td>
          <td class="tscclass" align="center"><p>eProsima: Jaime Martin Losa</p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><a class="reference internal" href="https://gvsc.army.mil"><img alt="GVSC logo" src="../_images/gvsc.png" style="height: 50px;" /></a></p></td>
          <td class="tscclass" align="center"><p>GVSC: Jerry Towler (SwRI)</p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><a class="reference internal" href="https://www.intel.com"><img alt="Intel logo" src="../_images/intel.svg" style="height: 50px;" /></a></p></td>
          <td class="tscclass" align="center"><p>Intel: Harold Yang</p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><a class="reference internal" href="https://intrinsic.ai/"><img alt="Intrinsic logo" src="../_images/intrinsic.png" style="height: 35px;" /></a></p></td>
          <td class="tscclass" align="center"><p>Intrinsic: Chris Lalancette</p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><a class="reference internal" href="https://www.irobot.com"><img alt="iRobot logo" src="../_images/irobot.png" style="height: 35px;" /></a></p></td>
          <td class="tscclass" align="center"><p>iRobot: Alberto Soragna</p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><a class="reference internal" href="https://www.microsoft.com"><img alt="Microsoft logo" src="../_images/microsoft.svg" style="height: 35px;" /></a></p></td>
          <td class="tscclass" align="center"><p>Microsoft: Lou Amadio</p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><a class="reference internal" href="https://www.openrobotics.org"><img alt="OSRF logo" src="../_images/openrobotics-logo-stacked.png" style="height: 35px;" /></a></p></td>
          <td class="tscclass" align="center"><p>OSRF: Geoff Biggs</p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><a class="reference internal" href="https://picknik.ai"><img alt="PickNik logo" src="../_images/picknik.png" style="height: 35px;" /></a></p></td>
          <td class="tscclass" align="center"><p>PickNik: Henning Kayser</p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><a class="reference internal" href="https://www.robotis.com"><img alt="ROBOTIS logo" src="../_images/robotis.png" style="height: 35px;" /></a></p></td>
          <td class="tscclass" align="center"><p>ROBOTIS: Will Son</p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><a class="reference internal" href="https://rosindustrial.org/about/description/"><img alt="ROS Industriallogo" src="../_images/ros-industrial.png" style="height: 35px;" /></a></p></td>
          <td class="tscclass" align="center"><p>ROS-Industrial: Matt Robinson</p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><a class="reference internal" href="https://www.samsung.com"><img alt="Samsung logo" src="../_images/samsung.svg" style="height: 25px;" /></a></p></td>
          <td class="tscclass" align="center"><p>Samsung: TBD</p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><a class="reference internal" href="https://www.sony.com"><img alt="Sony logo" src="../_images/sony.png" style="height: 60px;" /></a></p></td>
          <td class="tscclass" align="center"><p>Sony: Tomoya Fujita</p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><a class="reference internal" href="https://www.tri.global"><img alt="TRI logo" src="../_images/tri_logo_landscape-web.svg" style="height: 50px;" /></a></p></td>
          <td class="tscclass" align="center"><p>Toyota Research Institute: Ian McMahon</p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><a class="reference internal" href="https://www.windriver.com"><img alt="Wind River logo" src="../_images/windriver.png" style="height: 60px;" /></a></p></td>
          <td class="tscclass" align="center"><p>Wind River: Andrei Kholodnyi</p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><a class="reference internal" href="https://www.foxglove.dev"><img alt="Foxglove logo" src="../_images/foxglove.png" style="height: 60px;" /></a></p></td>
          <td class="tscclass" align="center"><p>Foxglove:  Adrian Macneil</p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><a class="reference internal" href="https://www.zettascale.tech/"><img alt="Zetta Scale logo" src="../_images/zettascale.png" style="height: 35px;" /></a></p></td>
          <td class="tscclass" align="center"><p>Zetta Scale: Angelo Corsaro</p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><img alt="ROS 2 logo" src="../_images/ros2_logo.png" style="height: 60px;" /></p></td>
          <td class="tscclass" align="center"><p>Community Representative: <a href="https://github.com/omichel"> Oliver Michel </a> </p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><img alt="ROS 2 logo" src="../_images/ros2_logo.png" style="height: 60px;" /></p></td>
          <td class="tscclass" align="center"><p>Community Representative: <a href="https://github.com/pmusau17"> Patrick Musau </a></p></td>
        </tr>
        <tr class="tscclass">
          <td class="tscclass" align="center"><p><img alt="ROS 2 logo" src="../_images/ros2_logo.png" style="height: 60px;" /></p></td>
          <td class="tscclass" align="center"><p>Community Representative:  <a href="https://github.com/fmrico"> Francisco Martin Rico </a></p></td>
        </tr>
      </tbody>
    </table>
    <br/>

If you are interested in joining the ROS 2 TSC, please inquire via info@openrobotics.org.

.. toctree::
   :maxdepth: 1

   Governance/ROS2-TSC-Charter
   Governance/ROS2-TSC-Intake-process
   Governance/How-To-Start-A-Community-Working-Group

Working Groups (WGs)
--------------------

As described in its :doc:`charter <Governance/ROS2-TSC-Charter>`, the TSC establishes working groups (WGs) to discuss and make progress on specific topics.

The current WGs are (12 as of 2021-01-12):

Client Libraries
^^^^^^^^^^^^^^^^

* Lead(s): Geoffrey Biggs, Alberto Soragna
* Note: **This working group is currently on hiatus. Meetings will resume at some point in the future TBD.**
* Resources:

 * Meeting invite group: `ros-client-libraries-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-client-libraries-working-group-invites>`_
 * `Meeting minutes and agendas <https://docs.google.com/document/d/1MAMQisfbITOR4eDyCBhTEaFJ3QBNW38S7Z7RpBBSSvg/edit>`_
 * Working group charter: https://github.com/ros2-client-libraries-wg/community
 * Discourse tag: `wg-client-libraries <https://discourse.ros.org/tags/wg-client-libraries>`_

Control
^^^^^^^

* Lead(s): Bence Magyar
* Resources:

 * Webite link: https://control.ros.org
 * Meeting invite group `ros-control-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-control-working-group-invites>`_
 * Discourse tag: `wg-ros2-control <https://discourse.ros.org/tags/wg-ros2-control>`_

Embedded Systems
^^^^^^^^^^^^^^^^

* Lead(s): Lara Moreno, Pablo Garrido
* Resources:

 * `2019-07-29 meeting notes <https://discourse.ros.org/uploads/short-url/z1caIm7m5IVP4cPJUwg3Chq36wO.pdf>`__
 * `2019-01-15 meeting notes <https://discourse.ros.org/t/ros2-embedded-sig-meeting-2/7243/5>`__
 * Meeting invite group `ros-embedded-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-embedded-working-group-invites>`_
 * Discourse tag: `wg-embedded <https://discourse.ros.org/tag/wg-embedded>`_

Middleware
^^^^^^^^^^

* Lead(s): William Woodall
* Resources:

 * Meeting invite group `ros-middleware-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-middleware-working-group-invites>`_
 * Discourse tag: `wg-middleware <https://discourse.ros.org/tag/wg-middleware>`_

Navigation
^^^^^^^^^^

* Lead(s): Steve Macenski
* Resources:

 * `2019-03-17 meeting notes <https://discourse.ros.org/t/ros2-navigation-wg-thursday-3-00-pm-pacific-gmt-7-00/7586/9>`__

 * Meeting invite group `ros-navigation-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-navigation-working-group-invites>`_
 * Discourse tag: `wg-navigation <https://discourse.ros.org/tag/wg-navigation>`_
 * Discourse Channel: `Navigation Stack <https://discourse.ros.org/c/navigation/44>`_
 * Slack Group: `Nav2 Slack <https://join.slack.com/t/navigation2/shared_invite/zt-uj428p0x-jKx8U7OzK1IOWp5TnDS2rA>`_

Manipulation
^^^^^^^^^^^^

* Lead(s): Henning Kayser
* Resources:

 * `About our working group meetings <https://discourse.ros.org/t/moveit-maintainer-meeting-all-invited-july-25th/9899>`__

 * Meeting invite group `ros-manipulation-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-manipulation-working-group-invites>`_
 * Discourse tag: `wg-moveit <https://discourse.ros.org/tag/moveit2>`_
 * Discourse Channel: `MoveIt <https://discourse.ros.org/c/moveit>`_

Real-time
^^^^^^^^^

* Lead(s): Andrei Kholodnyi, Carlos San Vicente
* Resources:

 * `Working group website <https://real-time-working-group.readthedocs.io/>`__
 * `Working Group Community <https://github.com/ros-realtime/community>`__
 * Meeting invite group `ros-real-time-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-real-time-working-group-invites>`_
 * Discourse tag: `wg-real-time <https://discourse.ros.org/tag/wg-real-time>`_
 * Matrix chat `+ros-realtime:matrix.org <https://matrix.to/#/+ros-realtime:matrix.org>`_

.. _Security Working Group:

Security
^^^^^^^^

* Lead(s): Jeremie Deray
* Resources:

 * `ROS 2 Security Working Group Community <https://github.com/ros-security/community>`__
 * Meeting invite group `ros-security-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-security-working-group-invites>`_
 * Discourse tag: `wg-security <https://discourse.ros.org/tag/wg-security>`_
 * Matrix chat `+rosorg-security:matrix.org <https://matrix.to/#/+rosorg-security:matrix.org>`_

Rosbag2 and Tooling
^^^^^^^^^^^^^^^^^^^

* Lead(s): Michael Orlov
* Resources:

 * `Charter <https://github.com/ros-tooling/community>`__
 * `Meeting Notes <https://docs.google.com/document/d/1Dsg_9XZQPhihpKQGQWMYTz2doGH4P2cAaNqr60cuNgw/edit>`__
 * Meeting invite group `ros-tooling-working-group-invites@googlegroups.com <https://groups.google.com/forum/#!forum/ros-tooling-working-group-invites>`_
 * Discourse tag: `wg-tooling <https://discourse.ros.org/tag/wg-tooling>`_
 * Matrix chat `+ros-tooling:matrix.org <https://matrix.to/#/+ros-tooling:matrix.org>`_


If you'd like to join an existing ROS 2 WG, please contact the appropriate group lead(s) directly.
If you'd like to create a new WG, please inquire via info@openrobotics.org.


Working Group Policies
----------------------

 * Meetings should be posted to the Google calendar as well as announced on Discourse.
 * Meetings should have notes and be posted to Discourse using appropriate working group tag.
 * For attending the groups meetings please join the associated google group to get invites automatically.

Upcoming ROS Events
-------------------

Upcoming Working group meetings can be found in this `Google Calendar <https://calendar.google.com/calendar/embed?src=agf3kajirket8khktupm9go748%40group.calendar.google.com&ctz=America%2FLos_Angeles>`_.
It can be accessed via `iCal <https://calendar.google.com/calendar/ical/agf3kajirket8khktupm9go748%40group.calendar.google.com/public/basic.ics>`_.

.. raw:: html

    <iframe src="https://calendar.google.com/calendar/embed?src=agf3kajirket8khktupm9go748%40group.calendar.google.com" style="border: 0" width="800" height="600" frameborder="0" scrolling="no"></iframe>



If you have an individual event or series of events that you'd like to post please contact info@openrobotics.org
