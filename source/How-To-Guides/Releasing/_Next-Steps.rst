Once your pull request has been submitted, usually within one or two days, one of the maintainers of rosdistro will review and merge your Pull Request.
<<<<<<< HEAD:source/How-To-Guides/Releasing/_Next-Steps.rst
If your package build is successful, in 24-48 hours your packages will become available in the **ros-testing** repository, where you can :doc:`test your pre-release binaries <../../../Installation/Testing>`.
=======
If your package build is successful, in 24-48 hours your packages will become available in the **ros-testing** repository, where you can :doc:`test your pre-release binaries <../../Debugging/Testing/Testing>`.
>>>>>>> 778d34f (3di | TOC update - move Testing articles into Debugging section (#7075)):source/Developer-Tools/Build/Releasing/_Next-Steps.rst

Approximately every two to four weeks, the distribution's release manager manually synchronizes the contents of ros-testing into the main ROS repository.
This is when your packages actually become available to the rest of the ROS community.
To get updates on when the next synchronization (sync) is coming, subscribe to the `Packaging and Release Management Category on Open Robotics Discourse <https://discourse.openrobotics.org/c/ros/release/16>`_.
