Development process for a release
=================================

.. contents:: Table of Contents
   :depth: 2
   :local:

Each ROS 2 distribution goes through a process of development more than a year long that begins prior to the release of the previous distribution.
Below is a high-level view of this development process.
There is no specific due date for the items in this process, but in general earlier items should be completed before later items can be completed.

For the progress through this process for a specific release, see that release's documentation page.

.. list-table::
   :class: release-process
   :header-rows: 1
   :widths: 30 70

   * - Item
     - Notes
   * - Find the ROS Boss
     - The "ROS Boss" is the person in charge of shepherding a distribution through the development, release, update, and EOL'ing stages of its life. They are chosen from the internal ROS 2 team at Open Robotics.
   * - Run process to choose the distribution name
     - The ROS Boss curates the process of choosing the distribution's name, using input from sources such as the community and potential naming conflicts.
   * - Create distribution's documentation page
     - Every distribution has a documentation page that lists its vital statistics, such as planned release date, EOL date, and significant changes since the previous release.
   * - Set release timeline
     - The final weeks leading up to release day (usually, World Turtle Day) are hectic and full of deadlines, such as when to freeze the default RMW implementation. These deadlines must be planned well in advance.
   * - Produce roadmap
     - While every contributor to ROS has their own planned features for each distribution, we try to maintain an overall roadmap of the new features and significant changes we expect to see in the distribution. The ROS Boss and the leader of the ROS 2 development team at Open Robotics work together with the ROS 2 TSC and other interested parties to produce a roadmap that is achievable in the time available and meets the needs of the ROS community.
   * - Announce roadmap
     - The list of planned features and significant changes is made public, via a GitHub issue that will track the progress on developing each item in the roadmap. Of course, this does not mean that the roadmap is fixed at this point, as development plans can change and we always (and frequently do) welcome new contributions even if they are not on the planned roadmap.
   * - Set target platforms and major dependencies
     - The target platforms, in terms of operating system, distribution and version, must be set far enough in advance for development work on the infrastructure (such as support in the build farm) to proceed. Similarly, the versions of each major dependency (which Python version, which compiler(s), which version of Eigen, etc.) must also be fixed. This is done via an update to `REP-2000 <https://ros.org/reps/rep-2000.html>`__.
   * - Add platform support to the build farm
     - The build farm is a critical part of the infrastructure supporting a ROS 2 distribution. It provides continuous integration facilities that help us maintain quality, and it builds the binary packages the community relies on to avoid building ROS 2 and packages from source. If the target platforms differ from the previous ROS 2 distribution, then the necessary support must be added to the build farm.
   * - Commission logo and related artwork
     - A well-loved part of every ROS 2 distribution (and ROS distribution!) is the logo. The logo is commissioned from a professional artist based on the chosen distribution name. Based on the logo, other artwork such as the turtlesim icon are also produced.
   * - Create mailing list for the distribution
     - Vital for making critical announcements, a mailing list must be set up to contact people interested in knowing something about the distribution, such as that their package is failing to build into a binary on the build farm.
   * - Create test cases
     - As the development process enters the final few months, testing begins in earnest. The integration test cases that will be used during the final stages of development must be produced and provided to the release team who will be responsible for executing them.
   * - Announce upcoming RMW freeze
     - The RMW freeze is the point at which the default RMW implementation for the new distribution is feature-frozen. This gives developers a stable target to test their packages with, which is particularly important for the client library developers, who need to know what features of the RMW layer will be available for use by client libraries.
   * - Upgrade dependency packages
     - Packages depended on by ROS but not ROS software and not available in the platform package manager (such as aptitude for Ubuntu), the so-called "vendor packages", must be updated to the versions specified in REP-2000 (or an appropriate version, for those not listed in REP-2000). This is particularly important on Windows.
   * - Create a detailed release plan
     - Planning for the final two months of the development process is performed. This produces a detailed test plan, timelines of when certain packages must be available, and so on. It enables the finding of dependencies between steps in the release process and finding people to perform each of those steps.
   * - Freeze RMW
     - The RMW implementation is now feature-frozen. In theory, it can now be exhaustively tested to ensure it is working correctly by release day.
   * - Announce upcoming overall freeze
     - The next freeze after freezing the RMW implementation is to freeze the distribution as a whole. This is the point at which the core ROS packages become feature-frozen, giving developers of non-core packages a stable target to test their packages against, and giving distribution testers something to test that won't change right after they've tested it.
   * - Freeze distribution
     - From this point on, no new features can be added to any of the core ROS packages. Only bug fixes for the (inevitable) bugs found during the intensive integration test phases of development can be incorporated into the codebase. This means that Rolling Ridley is effectively frozen, temperarily.
   * - Announce upcoming branch
     - The branching of the new ROS 2 distribution from Rolling Ridley is an important moment. It is worth preparing for.
   * - Announce upcoming beta
     - When the distribution enters beta, it is ready for wider testing by the ROS community. This beta happens soon after the distribution is branched from Rolling Ridely.
   * - Branch from Rolling Ridley
     - The new ROS 2 distribution is created by making a new branch from Rolling Ridley. In effect, the new distribution is born at this point in time. Meanwhile, Rolling Ridley is free from the development process and can roll on into the future, once again receiving new features.
   * - Add distribution to CI
     - The continuous integration system is updated to allow building using the new distribution's branches and core ROS packages. This means that package developers can run CI for their packages against the new distribution, rather than Rolling Ridley.
   * - Begin building interim testing tarballs
     - The elite team of testers who will put the new distribution through its paces need something to test without compiling ROS 2 from source constantly. The build farm is used to produce a set of tarballs containing the distribution at a point in time for the testers to test.
   * - Add distribution documentation
     - Detailed documentation about the distribution, such as the significant changes since the previous distribution, is added to the ROS 2 documentation site.
   * - Announce beta
     - The beta release of the distribution is made and the ROS community as a whole is invited to contribute to testing it (for those who aren't already doing so). At this point, the more testers the better, because the distribution needs to be put through as wide a range of scenarios as possible to find bugs before the release.
   * - Final release preparations
     - As the new distribution enters is absolutely-completely-everything-frozen phase, the final preparations are made for the release. These include things like producing binary packages using the build farm so there will be something to release.
   * - Release
     - The big day, which if all goes to plan coincides with World Turtle Day on May 23rd. The distribution's binary packages are made available in the release repository, and an announcement is made. Parties are held and the ROS 2 development team takes a well-earned break.
