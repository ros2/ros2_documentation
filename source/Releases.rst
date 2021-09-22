.. _Releases:

Distributions
=============

What is a Distribution?
-----------------------

A ROS distribution is a versioned set of ROS packages.
These are akin to Linux distributions (e.g. Ubuntu).
The purpose of the ROS distributions is to let developers work against a relatively stable codebase until they are ready to roll everything forward.
Therefore once a distribution is released, we try to limit changes to bug fixes and non-breaking improvements for the core packages (every thing under ros-desktop-full).
That generally applies to the whole community, but for "higher" level packages, the rules are less strict, and so it falls to the maintainers of a given package to avoid breaking changes.

.. _list_of_distributions:

List of Distributions
---------------------

Below is a list of current and historic ROS 2 distributions.
Rows in the table marked in green are the currently supported distributions.

.. toctree::
   :hidden:

   Releases/Alpha-Overview.rst
   Releases/Beta1-Overview.rst
   Releases/Beta2-Overview.rst
   Releases/Beta3-Overview.rst
   Releases/Release-Ardent-Apalone.rst
   Releases/Release-Bouncy-Bolson.rst
   Releases/Release-Crystal-Clemmys.rst
   Releases/Release-Dashing-Diademata.rst
   Releases/Release-Eloquent-Elusor.rst
   Releases/Release-Foxy-Fitzroy.rst
   Releases/Release-Galactic-Geochelone.rst
   Releases/Galactic-Geochelone-Complete-Changelog.rst
   Releases/Release-Humble-Hawksbill.rst
   Releases/Release-Rolling-Ridley.rst

.. raw:: html

   <!--
     This CSS overrides the styles of certain rows to mark them green, indicating they are supported releases.
     For the odd number rows, a line like the following must be used:

       .rst-content table.docutils:not(.field-list) tr:nth-child(1) td {background-color: #33cc66;}

     For the even number rows, a line like the following must be used:

       .rst-content tr:nth-child(2) {background-color: #33cc66;}

     No other combination I've found has worked.  Yes, this is extremely fragile.  No, I don't understand
     why it is like this.
   -->
   <style>
     .rst-content table.docutils:not(.field-list) tr:nth-child(1) td {background-color: #33cc66;}
     .rst-content tr:nth-child(2) {background-color: #33cc66;}
   </style>

.. |rolling| image:: Releases/rolling-small.png
   :alt: Rolling logo

.. |galactic| image:: Releases/galactic-small.png
   :alt: Galactic logo

.. |foxy| image:: Releases/foxy-small.png
   :alt: Foxy logo

.. |eloquent| image:: Releases/eloquent-small.png
   :alt: Eloquent logo

.. |dashing| image:: Releases/dashing-small.png
   :alt: Dashing logo

.. |crystal| image:: Releases/crystal-small.png
   :alt: Crystal logo

.. |bouncy| image:: Releases/bouncy-small.png
   :alt: Bouncy logo

.. |ardent| image:: Releases/ardent-small.png
   :alt: Ardent logo

.. list-table::
   :class: distros
   :header-rows: 1
   :widths: 35 30 20 15

   * - Distro
     - Release date
     - Logo
     - EOL date
   * - `Galactic Geochelone <Releases/Release-Galactic-Geochelone>`
     - May 23rd, 2021
     - |galactic|
     - November 2022
   * - `Foxy Fitzroy <Releases/Release-Foxy-Fitzroy>`
     - June 5th, 2020
     - |foxy|
     - May 2023
   * - `Eloquent Elusor <Releases/Release-Eloquent-Elusor>`
     - November 22nd, 2019
     - |eloquent|
     - November 2020
   * - `Dashing Diademata <Releases/Release-Dashing-Diademata>`
     - May 31st, 2019
     - |dashing|
     - May 2021
   * - `Crystal Clemmys <Releases/Release-Crystal-Clemmys>`
     - December 14th, 2018
     - |crystal|
     - December 2019
   * - `Bouncy Bolson <Releases/Release-Bouncy-Bolson>`
     - July 2nd, 2018
     - |bouncy|
     - July 2019
   * - `Ardent Apalone <Releases/Release-Ardent-Apalone>`
     - December 8th, 2017
     - |ardent|
     - December 2018
   * - `beta3 <Releases/Beta3-Overview>`
     - September 13th, 2017
     -
     - December 2017
   * - `beta2 <Releases/Beta2-Overview>`
     - July 5th, 2017
     -
     - September 2017
   * - `beta1 <Releases/Beta1-Overview>`
     - December 19th, 2016
     -
     - Jul 2017
   * - `alpha1 - alpha8 <Releases/Alpha-Overview>`
     - August 31th, 2015
     -
     - December 2016

Future Distributions
--------------------

For details on upcoming features see the :ref:`roadmap <Roadmap>`.

There is a new ROS 2 distribution released yearly on May 23rd (`World Turtle Day <https://www.worldturtleday.org/>`_).

.. list-table::
   :class: future-distros
   :header-rows: 1
   :widths: 25 30 20 25

   * - Distro
     - Release date
     - Supported for
     - Planned changes
   * - `Humble Hawksbill<Releases/Release-Humble-Hawksbill>`
     - May 2022
     - TBD
     - TBD


The expectation is to release new ROS 2 distributions once per year.

.. _rolling_distribution:

Rolling Distribution
--------------------

The Rolling distribution of ROS 2 serves as a staging area for future stable distributions of ROS 2 and as a collection of the most recent development releases.
Unlike most stable ROS 2 distributions which have an initial release, a support window during which they are updated, and a definite end of support (see :ref:`list_of_distributions` above) the Rolling distribution is continuously updated and is subject to in-place updates which will at times include breaking changes.

Packages released into the Rolling distribution will be automatically released into future stable distributions of ROS 2.
`Releasing a ROS 2 package <How-To-Guides/Releasing-a-ROS-2-package-with-bloom>` into the Rolling distribution follows the same procedures as all other ROS 2 distributions.

`ROS 2 Rolling Ridley <Releases/Release-Rolling-Ridley>` is the rolling development distribution of ROS 2 as proposed in `REP 2002 <https://www.ros.org/reps/rep-2002.html>`_.
It was first introduced in June 2020.

The Rolling distribution will receive frequent and possibly compatibility-breaking releases in core packages and we recommend most people use the most recent stable distribution instead.
Since new stable distributions will be created from snapshots of the Rolling distribution, package maintainers who want to make their packages available in future ROS 2 distributions can do so by releasing their packages into the Rolling distribution.
