Platform Support Tiers
======================

Platforms are defined as a combination of the OS, the architecture, and the RMW implementation.
If a platform / operating system provides multiple variants, (e.g. Ubuntu Desktop)  the ROS distro will be built for the desktop variant unless otherwise specified.
ROS 2 variants are defined in `REP 2001 <https://www.ros.org/reps/rep-2001.html>`_.

Tier 1
------

Tier 1 platforms are subjected to our unit test suite and other testing tools on a frequent basis including continuous integration jobs, nightly jobs, packaging jobs, and performance testing.
Errors or bugs discovered in these platforms are prioritized for correction by the development team.
Significant errors discovered in Tier 1 platforms can impact release dates and we strive to resolve all known high priority errors in Tier 1 platforms prior to new version releases.

Tier 2
------

Tier 2 platforms are subject to periodic CI testing which runs both builds and tests with publicly accessible results.
The CI is expected to be run at least within a week of relevant changes for the current state of the ROS distribution.
Package-level binary packages may not be provided but providing a downloadable archive of the built workspace is encouraged.
Errors may be present in released product versions for Tier 2 platforms.
Known errors in Tier 2 platforms will be addressed subject to resource availability on a best effort basis and may or may not be corrected prior to new version releases.
One or more entities should be committed to continuing support of the platform.

Tier 3
------

Tier 3 platforms are those for which community reports indicate that the release is functional.
The development team does not run the unit test suite or perform any other tests on platforms in Tier 3.
Installation instructions should be available and up-to-date in order for a platform to be listed in this category.
Community members may provide assistance with these platforms.
