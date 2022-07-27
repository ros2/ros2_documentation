Once your pull request has been submitted, one of the ROS developers will merge your request (this usually happens fairly quickly).
24-48 hours after that, your package will be built by the build farm and released into the building repository.
Packages built are periodically synchronized over to the `shadow-fixed <https://wiki.ros.org/ShadowRepository>`_ and public repositories, so it might take as long as a month before your package is available on the public ROS debian repositories (i.e. available via ``apt-get``).
To get updates on when the next synchronization (sync) is coming, subscribe to the `Packaging and Release Management Category on ROS Discourse <https://discourse.ros.org/c/release/16>`_.
