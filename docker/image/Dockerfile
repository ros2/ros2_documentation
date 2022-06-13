# This dockerfile is expecting to be run with the following build command
# (from the root of the git repository):
#
# docker build -f docker/image/Dockerfile .

FROM ubuntu:jammy

ARG user=rosindex
ARG uid=1000

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        git-all \
        graphviz \
        make \
        python3-doc8 \
        python3-docutils \
        python3-pip \
        python3-sphinx \
        python3-sphinx-copybutton \
        python3-sphinx-rtd-theme \
        python3-sphinx-tabs && \
    rm -rf /var/lib/apt/lists/*

RUN useradd -u $uid -m $user
ENV HOME=/home/$user
ENV PATH=/home/$user/.local/bin:$PATH
WORKDIR /tmp/doc_repository

USER $user

CMD pip3 install --no-warn-script-location --user -r requirements.txt -c constraints.txt && make multiversion
