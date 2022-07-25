.. redirect-from::

  Installation/Fedora-Development-Setup

Fedora (source)
===============

How to setup the development environment?
-----------------------------------------

The following system dependencies are required to build ROS 2 on Fedora. They can be installed with ``dnf`` as follows:

.. code-block:: bash

   sudo dnf install \
     cmake \
     cppcheck \
     eigen3-devel \
     gcc-c++ \
     liblsan \
     libXaw-devel \
     libyaml-devel \
     make \
     opencv-devel \
     patch \
     python3-colcon-common-extensions \
     python3-coverage \
     python3-devel \
     python3-empy \
     python3-nose \
     python3-pip \
     python3-pydocstyle \
     python3-pyparsing \
     python3-pytest \
     python3-pytest-cov \
     python3-pytest-mock \
     python3-pytest-runner \
     python3-rosdep \
     python3-setuptools \
     python3-vcstool \
     poco-devel \
     poco-foundation \
     python3-flake8 \
     python3-flake8-import-order \
     redhat-rpm-config \
     uncrustify \
     wget


With this done, you can follow the rest of the :ref:`instructions <rhel-dev-get-ros2-code>` to fetch and build ROS 2.

