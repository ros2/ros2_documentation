ROS Documentation
=================


This is the ROS 2 documentation site.


## Project structure

Under /source are the source files for the site.
In the root directory are the files required to locally build the site for testing.


## Build the site locally

### Prerequisites

You must have sphinx and the sphinx-tabs extension installed, preferably using pip:

```
pip3 install sphinx sphinx-tabs
```

For other distributions please refer to http://www.sphinx-doc.org/en/master/usage/installation.html


### Build the site

To build the site just execute:

```
make html
```

The site will be generated in the build/html directory. The main page is index.html
