.. _Help:

Contact
=======

.. _Using ROS Answers:

Support
-------

Different types of questions or discussions correspond to different avenues of communication;
check the descriptions below to ensure you choose the right method.

Need help troubleshooting your system?
First, search `ROS Answers <https://answers.ros.org>`__ to see if others have had similar issues, and if their solution works for you.

If not, ask a new question on `ROS Answers <https://answers.ros.org>`__.
Make sure to add tags, at the very least the ``ros2`` tag and the distro version you are running, e.g. ``{DISTRO}``.
If your question is related to the documentation here, add a tag like ``docs``, or more specifically, ``tutorials``.

Contributing support
^^^^^^^^^^^^^^^^^^^^

ROS 2 users come from a wide range of technical backgrounds, use a variety of different operating systems, and don’t necessarily have any prior experience with ROS (1 or 2).
So, it's important for users with any amount of experience to contribute support.

If you see an issue on `ROS Answers <https://answers.ros.org/questions/tags:ros2/>`__ that is similar to something you’ve run into yourself, please consider providing some pointers to what helped in your situation.
Don’t worry if you aren't sure if your response is correct.
Simply say so, and other community members will jump in if necessary.

Issues
------

If you identify bugs, have suggestions for improvements, or a question specific to one package, you can open an issue on GitHub.

For example, if you are following the :doc:`tutorials here <Tutorials>` and come across an instruction that doesn't work on your system,
you can open an issue in the `ros2_documentation <https://github.com/ros2/ros2_documentation>`__ repo.

You can search for individual ROS 2 repositories on `ROS 2's GitHub <https://github.com/ros2>`__.

Before opening an issue, check if other users have reported similar issues by searching across the ros2 and ament GitHub organizations: `example search query <https://github.com/search?q=user%3Aros2+user%3Aament+turtlesim&type=Issues>`__.

Next, check `ROS Answers <https://answers.ros.org/>`__ to see if someone else has asked your question or reported your issue.

If it has not been reported, feel free to open an issue in the appropriate repository tracker.
If it's not clear which tracker to use for a particular issue, file it in the `ros2/ros2 repository <https://github.com/ros2/ros2/issues>`__ and we'll have a look at it.

When filing an issue, please make sure to:

* Include enough information for another person to understand the issue.

Describe exactly what you were doing or are trying to do, and exactly what, if anything, went wrong.
If following a tutorial or online instructions provide a link to the specific instructions.

* Use a descriptive headline or subject line. Bad: "rviz doesn't work". Good: "Rviz crashing looking for missing .so after latest apt update"
* Include information about the exact platform, software, versions, and environment relevant to the problem. This includes how you installed the software (from binaries or from source) and which ROS middleware/DDS vendor you are using (if you know it).
* Any warnings or errors. Cut and paste them directly from the terminal window to which they were printed. Please do not re-type or include a screenshot.
* In case of a bug consider providing a `short, self contained, correct (compilable), example <http://sscce.org/>`__.
* When discussing any compiling/linking/installation issues, also provide the compiler version

As appropriate, also include your:

* ROS environment variables (env | grep ROS)
* Backtraces
* Relevant config files
* Graphics card model and driver version
* Ogre.log for rviz, if possible (run with rviz -l)
* Bag files and code samples that can reproduce the problem
* Gifs or movies to demonstrate the problem


Pull requests
-------------

When you feel comfortable enough to suggest a specific change directly to the code, you can submit a pull request.
Pull requests are welcome for any of `the ros2 repositories <https://github.com/ros2>`__.
See the :doc:`Contributing <The-ROS2-Project/Contributing>` page for more details and etiquette on how to contribute.

.. _Using ROS Discourse:

Discussion
----------

To start a discussion with other ROS 2 community members, visit the official `ROS Discourse <https://discourse.ros.org/>`__.
Content on the Discourse should be high-level;
it's not a place to get *questions* about code answered, but it would be suitable to start a conversation about best practices or improving standards.

Discussions about ROS 2 development and plans are happening on the `“Next Generation ROS” Discourse category <https://discourse.ros.org/c/ng-ros>`__.
Participating in these discussions is an important way to have a say on how different features of ROS 2 will work and be implemented.

The diverse community behind the ROS ecosystem is one of its greatest assets.
We encourage all members of the ROS community to participate in these design discussions so that we can leverage the experience of community members, and keep the varied use cases of ROS in mind.

Etiquette
----------

Assume 'good faith': It's easy to mis-interpret the meaning or tone of comments on the internet.
Assuming good faith gives the benefit of the doubt to those trying to help you, avoiding: insulting well meaning community members, and poisoning the mood.
Assuming 'good faith' when responding almost always works better even if the original response was not in fact in good faith.

Please don't send your question more than once: The question was seen.
If you didn't get a response then likely nobody has had time to answer you.
Alternatively, it could be that nobody knows the answer.
In any case, sending it again is poor form and akin to shouting and is likely to aggravate a large number of people.
This also applies to crossposting.
Try to pick the forum which you think matches best and ask there.
If you are referred to a new forum, provide a link to the old discussion.

On https://answers.ros.org you can edit your question to provide more details.
The more details that you include in your question the easier it is for others to help you find your solution which makes it more likely for you to get a response.

It's considered bad form to list your personal deadlines; community members answering questions also have them.

Do not beg for help.
If there is someone willing and able to help with your problem, you usually get a response.
Asking for faster answers will mostly have a negative effect.

Do not add unrelated content to posts.
The content of posts should be focused on the topic at hand and not include unrelated content.
Content, links, and images unrelated to the topic are considered spam.

For commercial posts, see also `this discussion <https://discourse.ros.org/t/sponsorship-notation-in-posts-on-ros-org/2078>`_.

Minimize references to content behind pay walls.
The content posted on `ROS Discourse <https://discourse.ros.org/>`__ and `ROS Answers <https://answers.ros.org/>`__ should "generally" be free and open to all users.
Links to content behind pay walls such as private journal articles, text books, and paid news websites, while helpful and relevant, may not be accessible to all users.
Where possible primary sources should be free and open with paid content playing a supporting role.

Single link posts are to be avoided.
Generally speaking, posting a single link answer is less helpful and can be easily confused with spam.
Moreover, links may degrade over time or be replaced.
Paraphrasing a link's content along with some contextual information and attribution is often much more helpful.

Private contact
---------------

If you'd like to contact us privately (e.g., if your question contains information sensitive to your organization or project, or if it's regarding a security issue), you can email us directly at ``ros@osrfoundation.org``.
