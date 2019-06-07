ROS Documentation
=================


This is the ROS2 documentation site.


## Project structure

Under /source are the source files for the site.
In the root directory are the files required to locally build the site for testing.


## Build the site locally

### Prerequisites

You must have sphinx installed. On Debian or Ubuntu you can install it using apt:

```
apt-get install python3-sphinx
```

For other distributions please refer to http://www.sphinx-doc.org/en/master/usage/installation.html

You also need a Sphinx extension called `recommonmark`, which allows you to write documents in Markdown rather than ReST, which can be installed with `pip`:

```
python3 -m pip install recommonmark
```

Installing it with `pip` should work on all platforms.

### Build the site

To build the site just execute:

```
make html
```

The site will be generated in the `build/html` directory.
The main page is `index.html`.
