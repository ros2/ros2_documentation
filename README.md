# ROS 2 Documentation

This repository contains the sources for the ROS 2 documentation that is hosted at [https://docs.ros.org/en](https://docs.ros.org/en).
The sources from this repository are built and uploaded to the site nightly by a [Jenkins job](https://build.ros2.org/job/doc_ros2doc/).

## Contributing to the documentation

Contributions to this site are most welcome.
Please see the [Contributing to ROS 2 Documentation](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Contributing-to-documentation.html) page to learn more.

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

For development we currently use Noble (Ubuntu 24.04) as our build platform.
And all python versions are pinned in the constraints file to make sure that things are reproducible.
To upgrade the system validate that things are working and then use `pip freeze > constraints.txt` to lock in the versions to upgrade.

## Building HTML

### Local development test

For local testing of the current tree use:

`make html`

`sensible-browser build/html/index.html`

### Live-reload local development

To iterate on documentation without manually rebuilding and refreshing the browser, use [`sphinx-autobuild`](https://github.com/sphinx-doc/sphinx-autobuild).
It watches the source files, rebuilds incrementally on save, and serves the result with automatic browser reload.

`sphinx-autobuild` is installed as part of `requirements.txt`.
Start the live server with:

```
make serve
```

Then open `http://localhost:2022` in a browser.

The `serve` target binds to `0.0.0.0:2022` by default (a little ROS 2 vibe) so the server is reachable through devcontainer / port forwarding.
Override the bind address or port if needed:

```
make serve LIVE_HOST=127.0.0.1 LIVE_PORT=8080
```

### Spelling Check

To check the spelling, use:

`make spellcheck`

> [!NOTE]
> If that detects specific words that need to be ignored, add it to [codespell_whitelist](./codespell_whitelist.txt). \
> To include any custom corrections that are to be applied, add it to [codespell_dictionary](./codespell_dictionary.txt).

### Deployment test

To test building the multisite version deployed to the website use:

`make multiversion`

`sensible-browser build/html/rolling/index.html`

**NB:** This will ignore local workspace changes and build from the branches.

### Faster (parallel) builds

Both `make html` and `make multiversion` build Sphinx in parallel by default,
using one worker per CPU core (Sphinx's `-j auto`). This is handled inside
Sphinx, so it works the same on Linux, macOS, and Windows — note that a plain
`make -j` does **not** help, because each build is a single Sphinx invocation.

To pin the number of workers instead of auto-detecting, set `JOBS`:

```
make html JOBS=8
make multiversion JOBS=8
```

**NB:** For `make multiversion`, `JOBS` parallelizes the work *within* each
branch's build; the branches themselves are still built one after another.

### Note for Windows (WSL) Users

When building the documentation on windows using WSL, it is recommended to clone and work with this repository inside the Linux filesystem (for example, under `/home/<user>/`) rather than under `/mnt/c`.

Working under `/mnt/c` can lead to slower builds and filesystem-related issues with Sphinx and ROS tooling.
