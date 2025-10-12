# This dockerfile is expecting to be run with the following build command
# (from the root of the git repository):
#
# $ docker build -f docker/image/Dockerfile --build-arg user=$(id -un) --build-arg uid=$(id -u) -t ros2_documentation .
#
# Then, to use the image to build the docs:
#
# $ docker run --rm -v $(pwd):/tmp/doc_repository ros2_documentation

FROM ubuntu:noble

ARG user=rosindex
ARG uid=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL=/bin/bash

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $uid ; then userdel `id -un $uid` ; fi

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        git-all \
        graphviz \
        locales \
        make \
        python3-pip && \
    rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN useradd -u $uid -m $user
ENV HOME=/home/$user
ENV PATH=/home/$user/.local/bin:$PATH
WORKDIR /tmp/doc_repository

USER $user

# https://build.ros.org/job/doc_ros2doc/ relies on "make multiversion" to build the official documentation.
# Do not remove "make multiversion" from CMD, as it is the entrypoint for the container.
# The constraints.txt file was generated with:
#   $ cd ros2_documentation
#   $ docker run -v $PWD:/ros2_documentation -it ubuntu:24.04
#   (docker)$ cd /ros2_documentation
#   (docker)$ apt update
#   (docker)$ apt install -y python3-pip python3-venv
#   (docker)$ python3 -m venv venv-constraints
#   (docker)$ . ./venv-constraints/bin/activate
#   (docker)$ pip3 install -U -r requirements.txt
#   (docker)$ pip3 freeze > constraints.txt
CMD ["bash", "-c", "pip3 install --no-warn-script-location --user --break-system-packages -r requirements.txt -c constraints.txt && make multiversion"]
