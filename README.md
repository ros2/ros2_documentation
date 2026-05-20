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

For development we currently use Noble (Ubuntu 24.04) as our build platform.
And all python versions are pinned in the constraints file to make sure that things are reproducible.
To upgrade the system validate that things are working and then use `pip freeze > constraints.txt` to lock in the versions to upgrade.

## Building HTML

### Local development test

For local testing of the current tree use:

`make html`

`sensible-browser build/html/index.html`

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

### Pagefind search index

After `make html` or `make multiversion`, run [Pagefind](https://pagefind.app/) so the built HTML under `build/html` is indexed and `build/html/pagefind/` is written (search bundle and Component UI assets). From the repo root:

`make pagefind`

Or use convenience targets that run Sphinx and Pagefind in one step:

- `make html-search` — `make html` then `make pagefind`
- `make multiversion-search` — `make multiversion` then `make pagefind`

Plain `make html` and `make multiversion` do **not** run Pagefind (Node.js is only required when you index search).

This requires **Node.js** (for `npx`). Pin the CLI with `PAGEFIND_VERSION` in the Makefile if needed.

To preview search locally, serve the site over HTTP (Pagefind may not load from `file://`), for example from the repo root:

`python -m http.server 8000 --directory build/html`

Then open `http://localhost:8000/` in a browser.

#### Search results page verification

After `make html` and `make pagefind`, serve `build/html` over HTTP and check:

1. **Direct URL** — Open `http://localhost:8000/search.html?q=tutorial` (or the same path under a distro prefix for multiversion builds). The input should show the query and results should load (not stay empty or skeleton-only).
2. **Modal redirect** — From a nested page (e.g. a tutorial), open the sidebar search modal (Ctrl/Cmd+K), type a term, press Enter. You should land on the search page with `?q=` set and matching results visible.
3. **Empty query** — Open `search.html` with no `q` parameter. The page should load without errors; no search is run until you type in the input.
4. **Result metadata** — Search for `Ubuntu deb` and open a result card. Metadata labels (e.g. Area, Content Type, Experience) should match that page’s `<head>` `<meta name="..." content="...">` tags from its `.. meta::` block (e.g. `area: installation` on the Ubuntu deb install page), not URL-path guesses.

In DevTools Network, confirm `pagefind/` bundle requests return 200 (not 404).

The production [Jenkins doc job](https://build.ros.org/job/doc_ros2doc) should run the same `pagefind` step on `build/html` after Sphinx so deployed pages include the search bundle.


### Note for Windows (WSL) Users

When building the documentation on windows using WSL, it is recommended to clone and work with this repository inside the Linux filesystem (for example, under `/home/<user>/`) rather than under `/mnt/c`.

Working under `/mnt/c` can lead to slower builds and filesystem-related issues with Sphinx and ROS tooling.
