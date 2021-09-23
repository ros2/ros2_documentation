How to Start a Community Working Group
======================================

.. contents:: Table of Contents
   :depth: 1
   :local:


A ROS working group is a great way to get a bunch of like minded people together
to make regular progress towards a shared goal.
So what are the steps to making a working group?

#. First try to find a couple of like-minded individuals who are willing to
   regularly commit their time to a particular area of ROS development. We
   suggest having at minimum three people who can help orchestrate the working
   group.

#. Draft up a one page summary of your working group. This summary should
   broadly cover what topics you would like to cover and what goals you would
   like to accomplish. Take a look at the other working groups and make sure
   that you don’t overlap too much.

#. Once you have your ideas roughly distilled take it to the ROS community to
   find more like minded members. We generally recommend making a post on `ROS
   discourse <https://discourse.ros.org>`__ on the general channel with the title “Proposal for ROS X Working
   Group”. Include your summary and your current collaborators. You should use
   this thread to:

   a) Finalize the subjects covered by your working group.
   b) Recruit members for your working group.
   c) Set the agenda for your first couple of working group meetings.
   d) Pick a regular time for your working group meetings (we find that early in
      the morning PST, generally works for most of the world).  At this point
      you should also make a discourse tag (you use these when you make your
      post.  We suggest something like: “wg-<topic>”.

#. Once you have recruited your working group members you’ll have some leg work
   to do. At this point you should have a list of working group members. Create
   a google group by going to Google groups
   (https://groups.google.com/my-groups) and adding your members.
   This mailing list should be named ``<dashed-name>-working-group-invites@googlegroups.com``.
   It should be setup such that:

     * Anyone can join
     * No one can post (aka only owners)
     * Anyone can view members

   The mailing list will be used to distribute the invitations only, any communications should use `ROS discourse <https://discourse.ros.org>`__ with the working group's tag.
   Please add tfoote@osrfoundation.org as an owner of the mailing list to help with administration.

#. Now that you have an e-mail group you can associate that group with the ROS
   events calendar. The calendar is where you will schedule your working group
   meetings. To join the calendar, send an e-mail request to Kat
   (kat@openrobotics.org) or Tully (tully@openrobotics.org).

#. To formalize your group you need to create a working group charter and a git
   repository to keep your documents and code. We have a working group template
   that can be found `here
   <https://github.com/ros2/tsc_working_group_governance_template>`_. Anyone in
   the group can host the template or you can request that your group be added
   to the Open Robotics / OSRF organization. The decision is up to the working group.

#. Now that you have a working group charter, e-mail group, and source
   repository, you can add all of that information to the :ref:`ROS 2 project governance
   website <Governance>` by sending a pull request to https://github.com/ros2/ros2_documentation.

#. Now it is time to schedule your first official working group meeting! We
   recommend announcing the meeting on ROS Discourse approximately one week
   before the meeting date. Please include the following in your an announcement:

   a) An agenda for the meeting.
   b) The date and time for the meeting (protip: discourse has a date/time feature
      that can auto adjust for each user’s timezone).
   c) URLs / Links / Contact Info for the meeting. Open Robotics uses Google
      Hangouts, but you can use whatever technology you would like.

   When you schedule your meeting, create an event on the ROS Calendar and invite the Google group so that anyone interested can sign up for automatic invitations.

#. Have your meetings! Regularly!

   a) Since we are a global community it can help to record your meetings. If
      you have an invited lecture we highly recommend recording it for the
      benefit of the whole community. Make sure the group / lecturer consents to
      being recorded, and it is wise to do a test run before the event.

      i) On Linux SimpleScreenRecorder works well but you may need to iterate to
         find the best combination of video conferencing platform, screen
         recorder, and settings.

      ii) Once your meeting is over, find somewhere online to save the
          video. Feel free to use your own Youtube channel to post the video;
          alternatively Open Robotics can host the video on their Vimeo account
          by sending a download link to kat@openrobotics.org.

  b) Make sure to take notes! You should nominate a scribe or note taker who
     summarizes the meeting and posts the results of the meeting to discourse.
