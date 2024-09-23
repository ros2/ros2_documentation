# ROS 2 Documentation

This repository contains the sources for the ROS 2 documentation that is hosted at [https://docs.ros.org/en](https://docs.ros.org/en).
The sources from this repository are built and uploaded to the site nightly by a [Jenkins job](https://build.ros.org/job/doc_ros2doc).

## Contributing to the documentation

Contributions to this site are most welcome.
Please see the [Contributing to ROS 2 Documentation](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Contributing-To-ROS-2-Documentation.html) page to learn more.

## Contributing to ROS 2

To contribute to the ROS 2 source code project please refer to the [ROS 2 contributing guidelines](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing.html).

## Prerequisites

To build this you need to install

* make
* graphviz

With [venv](https://docs.python.org/3/library/venv.html)

```
# activate the venv
python3 -m venv ros2doc

# activate venv
source ros2doc/bin/activate

# install required packages
pip install -r requirements.txt -c constraints.txt

# deactivate the venv
(ros2doc) deactivate
```

### Pinned versions

For development we currently use Noble as our build platform.
And all python versions are pinned in the constraints file to make sure that things are reproducible.
To upgrade the system validate that things are working and then use `pip freeze > constraints.txt` to lock in the versions to upgrade.

## Building HTML

### Local development test

For local testing of the current tree use:

`make html`

`sensible-browser build/html/index.html`

### Deployment test

To test building the multisite version deployed to the website use:

`make multiversion`

`sensible-browser build/html/rolling/index.html`

**NB:** This will ignore local workspace changes and build from the branches.
