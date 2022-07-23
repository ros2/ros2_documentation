Once your pull request has been submitted, one of the ROS developers will merge your request (this usually happens fairly quickly).
24-48 hours after that, your package will be built by the build farm and released into the building repository.
Packages built are periodically synchronized over to the `shadow-fixed <https://wiki.ros.org/ShadowRepository>`_ and public repositories, so it might take as long as a month before your package is available on the public ROS debian repositories (i.e. available via ``apt-get``).
To get updates on when the next synchronization (sync) is coming, check the `ROS discussion forums <https://discourse.ros.org/>`_.

Individual build details are on the Jenkins build farm `build.ros2.org <http://build.ros2.org/>`__.
Check `ROS {DISTRO} Default Package Status <http://repo.ros2.org/status_page/ros_{DISTRO}_default.html>`__ to see status of released packages.
