# ROS 2 Roadmap

For more information on the design of ROS 2 please see [design.ros2.org](http://design.ros2.org).
The core code for ROS 2 is on the [ros2 github organization](https://github.com/ros2).
The Discourse forum/mailing list for discussing ROS 2 design is [ng-ros](https://discourse.ros.org/c/ng-ros).
Questions should be asked on [ROS answers](https://answers.ros.org), make sure to include at least the `ros2` tag and the rosdistro version you are running, e.g. `ardent`.

## Planned upcoming releases

This is a list of the features targeted for development in the future.

*Subject to change.*

### Next release - Crystal Clemmys (December 2018)

The [meta ticket](https://github.com/ros2/ros2/issues/529) on GitHub contains an up-to-date state of the ongoing high level tasks as well as references specific tickets with more details.

- Support parameters in rclpy
- Time: support for simtime, clock / time in Python, rates / timers using specific clocks, using same clock in tf2
- Port image_transport to ROS 2
- Launch: support life cycle, support components, nesting, launch in containers, pass parameters
- Rosbag: record / playback binary messages, C++ API, Python CLI
- Port gazebo_ros_pkgs to ROS 2: the "main" plugins
- Port robot_web_tools
- Actions: code generation, server / client API
- IDL format: using `IDL 4.2` to specify ROS interfaces (msgs, srvs, actions), leverage new features like grouping and various annotations (comments, units)
- Improve MISRA compliance, audit memory management
- Buildfarm: keep non-latest .deb files, "incremental" CI job building of fat archive, CI jobs testing branches across repos, support nvidia-docker for testing with e.g. Gazebo
- Documentation infrastructure: rosindex-like page to browse/search metadata of ROS 2 packages, support generating docs from e.g. `.rst` files
- Performance testing

### Future (in no specific order)

#### Design / Concept

- Support for non-ASCII strings in messages / services
- Progress on migration plan
- Reconsider 1-to-1 mapping of ROS nodes to DDS participants
- Python-based launch with stable API, introspectable, optional XML frontend
- Make `ament_cmake` available in ROS 1 and/or `catkin` available in ROS 2

#### Infrastructure and tools

- Building
  - Support to generate "fat" packages / archives
  - Windows and Mac OS packages
- Documentation
  - Platform for documentation (like wiki.ros.org), allow easy contributions as well as optionally facilitate a review process
  - Support for `doc` jobs on the [ROS 2 buildfarm](http://build.ros2.org)
  - Consider consolidating with design.ros2.org
  - Provide three different kinds of content:
    - "demos" to show features and cover them with tests
    - "examples" to show a simple/minimalistic usage which might have multiple ways to do something
    - "tutorials" which contain more comments and anchors for the wiki (teaching one recommended way)

#### New features

The trailing stars indicate the rough effort: 1 star for small, 2 stars for medium, 3 stars for large.

- Expose matched publisher / subscriber count (rather than only based on the topic name) [**]
  - E.g. a best effort publisher and a reliable subscriber, the current API returns 1 subscriber for the topic, so the publisher might do computational intensive work even though no subscriber needs the messages
  - Querying this information requires a publisher / subscriber handle (against which the matched count is determined)
  - Requires knowledge of the rmw interface which needs to be extended
- Logging improvements [* / **]
  - Configuration specified in a file
  - Log to file
  - Log to `rosout` topic
  - The API has a single callback at the moment, a composition callback could implement the multiple additional "destinations"
  - Most of the implementation should be done in `rcl`
  - C++ stream operators
- Additional Graph API features [** / ***]
  - a la ROS 1 Master API: http://wiki.ros.org/ROS/Master_API
  - Event-based notification
  - Requires knowledge of the rmw interface which needs to be extended
- Remapping [** / ***]
  - Dynamic remapping and aliasing through a Service interface
- Type masquerading [***]
  - a la ROS 1's message traits: http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes
  - Requires knowledge of the typesupport system
- Expand on real-time safety [***]
  - With FastRTPS
  - For services, clients, and parameters
  - Support deterministic ordering of executables in Executor (fair scheduling)
  - Expose more quality of service parameters related to real-time performance
  - Real-time-safe intra-process messaging
- Multi-robot supporting features and demos [***]
  - Undesired that all nodes across all robots share the same domain (and discover each other)
  - Design how to “partition” the system
- Implement C client library `rclc` [**]
- Support more DDS / RTPS implementations:
  - Connext dynamic [*]
  - RTI's micro implementation [*]
  - Eclipse Cyclone DDS (former ADLINK OpenSplice) [*]
- security improvements:
  - more granularity in security configuration (allow authentication only, authentication and encryption, etc) [*]
  - extend access control permission generation to support services [*]
  - integrate DDS-Security logging plugin (unified way to aggregate security events and report them to the users through a ROS interface) [* *]
  - key storage security (right now, keys are just stored in the filesystem) [* *]
  - more user friendly interface (make it easier to specify security config). Maybe a Qt GUI? This GUI could also assist in distributing keys somehow. [* * *]
  - A way to say "please secure this running system" with some UI that would auto-generate keys and policies for everything that is currently running. [* * *]
  - If there are hardware-specific features for securing keys or accelerating encryption/signing messages, that could be interesting to add to DDS/RTPS implementations that don't use it already. [* * *]

#### Port of existing ROS 1 functionality

- Perception metapackage
  - Image pipeline
  - Improvements to the intra process comm. to reduce latency / overhead
- Navigation
  - `robot_pose_ekf` or `robot_localization`
  - `move_base`
  - Working group https://discourse.ros.org/t/ros2-navigation-working-group-kick-off/5559
- MoveIt
  - Needs Actions
  - Moveit Maintainers are tracking: https://discourse.ros.org/t/moveit-maintainer-meeting-recap-july-25th-2018/5504
- Rqt
  - `python_qt_binding` needs support for Python 3 (nothing ROS specific in this package) [*]
  - `rqt_gui` / `rqt_gui_cpp` need to be migrated to use ROS 2 API [*]
  - convert each plugin [* each when dependencies are available]
  - User-friendly plugin registration
- Diagnostics

#### Reducing Technical Debt

- Extend testing and resolve bugs in the current code base
  - Waitset inconsistency
  - Multi-threading problems with components
  - Reduce overhead / latency of intra-process communication
- Fix flaky tests.
- Ability to run (all) unit tests with tools e.g. valgrind
- API review
- Synchronize / reconcile design docs with the implementation.
  - Pre-release retrospective review (APIs, docs, etc.)
- Address / classify pending tickets
- Address TODOs in code / docs

## Past releases

See [list of releases](Releases).
