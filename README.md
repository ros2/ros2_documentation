# ROS 2 Documentation

This repository contains the sources for the ROS 2 documentation that is hosted at [https://docs.ros.org/en](https://docs.ros.org/en).
The sources from this repository are built and uploaded to the site nightly by a [Jenkins job](https://build.ros.org/job/doc_ros2doc).

## Contributing to the documentation

Contributions to this site are most welcome.
Please be sure to read the below sections carefully before contributing.

The site is built using [Sphinx](https://www.sphinx-doc.org/en/master/), and more particularly using [Sphinx multiversion](https://holzhaus.github.io/sphinx-multiversion/master/index.html).

### Installing prerequisites

```
pip3 install --user --upgrade -r requirements.txt
```

### Branch structure

This repository is setup with one branch per ROS 2 distribution to handle differences between the distributions.
If a change is common to all ROS 2 distributions, it should be made to the `rolling` branch (and then will be backported as appropriate).
If a change is specific to a particular ROS 2 distribution, it should be made to the respective branch.

### Source structure

The source files for the site are all located under the `source` subdirectory.
Templates for various sphinx plugins are located under `source/_templates`.
The root directory contains configuration and files required to locally build the site for testing.

### Building the site

To build the site for just this branch, type `make html` at the top-level of the repository.
This is the recommended way to test out local changes.

To build the site for all branches, type `make multiversion` from the `rolling` branch.
This has two drawbacks:

1. The multiversion plugin doesn't understand how to do incremental builds, so it always rebuilds everything.  This can be slow.
1. When typing `make multiversion`, it will always check out exactly the branches listed in the `conf.py` file.  That means that local changes will not be shown.

To show local changes in the multiversion output, you must first commit the changes to a local branch.
Then you must edit the [conf.py](./conf.py) file and change the `smv_branch_whitelist` variable to point to your branch.

### Macros

Macros can be used to simplify writing documentation that targets multiple distributions.

Use a macro by including the macro name in curly braces.
For example, when generating the docs for Rolling on the `rolling` branch:

| Use | Becomes (for Rolling) | Example |
|-----|-----------------------|---------|
| {DISTRO} | rolling | ros-{DISTRO}-pkg |
| {DISTRO_TITLE} | Rolling | ROS 2 {DISTRO_TITLE} |
| {DISTRO_TITLE_FULL} | Rolling Ridley | ROS 2 {DISTRO_TITLE_FULL} |
| {REPOS_FILE_BRANCH} | master | git checkout {REPOS_FILE_BRANCH} |

The same file can be used on multiple branches (i.e., for multiple distros) and the generated content will be distro-specific.

## Contributing to ROS 2

To contribute to the ROS 2 source code project please refer to the [ROS 2 contributing guidelines](https://docs.ros.org/en/rolling/Contributing.html).
