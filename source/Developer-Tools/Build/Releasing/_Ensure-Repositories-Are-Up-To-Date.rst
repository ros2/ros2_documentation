.. redirect-from::

  How-To-Guides/Releasing/_Ensure-Repositories-Are-Up-To-Date

.. meta::
   :contentType: how-to
   :experience:
   :area: builds, tools
   :distribution: {DISTRO}
   :product: {PRODUCT}

Ensuring repositories are up-to-date - how-to
=============================================

.. short-description::
   Keeping repositories current helps avoid release problems caused by missing commits or working on the wrong branch.
   This article describes the repository checks to complete before continuing with a ROS release.
   After you follow these steps, your local clone and remote repository will be ready for the next release action.

.. showmeta::
    :order: area, contentType, experience
    :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Table of Contents
    :depth: 2
    :local:

Summary
-------

Before continuing with a release, make sure the repository is available on a remote hosting service such as GitHub.
Use a local clone of that repository and confirm that you are working on the correct branch.

Make sure that:

* Your repository is hosted on a remote such as GitHub.

* You have a clone of the repository on your computer and are on the right branch.

* Both the remote repository and your clone are up-to-date.
