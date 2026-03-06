Release Schedule
================

Frequency
---------
New ROS 2 releases are produced every **every 12 months**.
The rationale is that a shorter cycle (like 6 months) results in significant overhead and potentially many active releases at the same time (assuming they have the same support length).
On the other hand a longer cycle (like 2 years) is too long for users to wait for new features to be available in a ROS 2 release.

Targeted Platforms
------------------

Since non-LTS (long term support) Ubuntu releases are only supported for 9 months, ROS 2 will not target those non-LTS Ubuntu releases.
A single ROS 2 distribution will only have full Tier 1 support for a **single** Ubuntu LTS.
The rationale is that fully supporting two Ubuntu LTS versions is a tremendous overhead for our maintainers as there may be upstream dependencies that differ by up to two years .
On a case-by-case basis, a ROS 2 distribution may support an older Ubuntu LTS distribution as a Tier 3, community-supported platform.

Since both macOS (or at least brew) and Windows are rolling platforms, we aim to support the latest version available at the time of a ROS 2 distribution's release.
For Debian we also aim to target the latest stable version; however, if that version is two years behind the Ubuntu version then it might not be possible.

Support
-------

LTS releases
^^^^^^^^^^^^

Since Ubuntu LTS releases come with **5 years** of standard support, we aim for each ROS LTS distribution to have a similar support lifetime.
In even years new ROS 2 releases will happen one month after the Ubuntu LTS has been released (which usually means a ROS 2 release in May).
The ROS 2 release will be supported until the end of the standard support window of the Ubuntu LTS release, which is 4 years and 11 months from the ROS 2 release date.

Non-LTS Releases
^^^^^^^^^^^^^^^^

In order to provide frequent releases to the community, in odd years a non-LTS ROS 2 release will be published.
It will always target the same Ubuntu LTS as the previous ROS 2 LTS release but will only be supported for **1.5 years**.
This duration ensures that the non-LTS will overlap with the next ROS LTS release by 6 months to provide a long enough transition window.

Releases and support duration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- May 2021: G Turtle: non-LTS release, supported for 1.5 years
- May 2022: H Turtle: LTS release, supported for 5 years
- May 2023: I Turtle: non-LTS release, supported for 1.5 years
- May 2024: J Turtle: LTS release, supported for 5 years
- and so on, alternating annually between LTS and non-LTS releases
