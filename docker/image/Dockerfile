# This dockerfile is expecting to be run with the following build command
# (from the root of the git repository):
#
# docker build -f docker/image/Dockerfile .

FROM ubuntu:jammy

ARG user=rosindex
ARG uid=1000

ENV DEBIAN_FRONTEND noninteractive
ENV SHELL /bin/bash

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        git-all \
        graphviz \
        make \
        python3-pip locales && \
    rm -rf /var/lib/apt/lists/*

RUN useradd -u $uid -m $user
ENV HOME=/home/$user
ENV PATH=/home/$user/.local/bin:$PATH
ENV LANG=en_US.UTF-8
WORKDIR /tmp/doc_repository

USER $user

CMD locale-gen en_US en_US.UTF-8
CMD update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
CMD pip3 install --no-warn-script-location --user -r requirements.txt -c constraints.txt && make multiversion
