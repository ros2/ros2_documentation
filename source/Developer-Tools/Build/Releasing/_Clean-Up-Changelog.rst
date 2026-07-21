.. redirect-from::

    How-To-Guides/Releasing/_Clean-Up-Changelog

.. meta::
   :contentType: how-to
   :experience:
   :area: builds, tools
   :distribution: {DISTRO}
   :product: {PRODUCT}

Cleaning up changelog files - how-to
====================================

.. short-description::
   Generated changelog entries often need editing before a package release.
   This article explains how to clean up the auto-generated Forthcoming section in each CHANGELOG.rst file.
   After you follow it, the changelog will describe the notable package changes clearly and be ready to commit.

.. showmeta::
   :order: area, contentType, experience
   :labels: area=Area, contentType=Content type, experience=Level

.. contents:: Table of Contents
   :depth: 2
   :local:

Open all ``CHANGELOG.rst`` files in an editor.
You will see that ``catkin_generate_changelog`` has auto-generated a forthcoming section with notes from commit messages:

.. code-block:: rst

   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   Changelog for package your_package
   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

   Forthcoming
   -----------
   * you can modify this commit message
   * and this

Clean up the list of commit messages to concisely convey the notable changes that have been made to the packages since the last release, and **commit all the CHANGELOG.rst files.**
Do not modify the ``Forthcoming`` header.
