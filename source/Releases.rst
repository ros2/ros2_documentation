.. _Releases:

Distributions
=============

What is a Distribution?
-----------------------

See `wiki.ros.org/Distributions <https://wiki.ros.org/Distributions>`_.

.. _list_of_distributions:

List of Distributions
---------------------

.. toctree::
   :hidden:
   :glob:

   Releases/*

.. raw:: html

   <style>
     .distros td {border: 0px;}
     .distros tbody tr {background-color: #c0c0c0;}
     .distros tbody tr:nth-child(1), .distros tbody tr:nth-child(2), .distros tbody tr:nth-child(3) {background-color: #33cc66;}
     .distros td {vertical-align: middle;}
   </style>

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
   * - `Foxy Fitzroy <Releases/Release-Foxy-Fitzroy>`
     - June 5th, 2020
     - |foxy|
     - May 2023
   * - `Eloquent Elusor <Releases/Release-Eloquent-Elusor>`
     - Nov 22nd, 2019
     - |eloquent|
     - Nov 2020
   * - `Dashing Diademata <Releases/Release-Dashing-Diademata>`
     - May 31st, 2019
     - |dashing|
     - May 2021
   * - `Crystal Clemmys <Releases/Release-Crystal-Clemmys>`
     - December 14th, 2018
     - |crystal|
     - Dec 2019
   * - `Bouncy Bolson <Releases/Release-Bouncy-Bolson>`
     - July 2nd, 2018
     - |bouncy|
     - Jul 2019
   * - `Ardent Apalone <Releases/Release-Ardent-Apalone>`
     - December 8th, 2017
     - |ardent|
     - Dec 2018
   * - `beta3 <Releases/Beta3-Overview>`
     - September 13th, 2017
     -
     - Dec 2017
   * - `beta2 <Releases/Beta2-Overview>`
     - July 5th, 2017
     -
     - Sep 2017
   * - `beta1 <Releases/Beta1-Overview>`
     - December 19th, 2016
     -
     - Jul 2017
   * - `alpha1 - alpha8 <Releases/Alpha-Overview>`
     - August 31th, 2015
     -
     - Dec 2016

Distribution Details
~~~~~~~~~~~~~~~~~~~~

For details on the distributions see each release page.
For the supported platforms and versions of common dependencies and other considerations, see the official ROS 2 Target Platforms `REP 2000 <https://www.ros.org/reps/rep-2000.html>`_.

Future Distributions
--------------------

For details on upcoming features see the :ref:`roadmap <Roadmap>`.

Currently there is a new ROS 2 distribution roughly every 6 months.
The following information is a best estimate and is subject to change.

.. raw:: html

   <style>
     .future-distros td {vertical-align: middle;}
   </style>

.. list-table::
   :class: future-distros
   :header-rows: 1
   :widths: 25 30 20 25

   * - Distro
     - Release date
     - Supported for
     - Planned changes
   * - `Galactic Geochelone <Releases/Release-Galactic-Geochelone>`
     - May 2021
     - TBD
     - TBD


The expectation is to release new ROS 2 distributions once per year.

.. _rolling_distribution:

Rolling Distribution
--------------------

The Rolling distribution of ROS 2 serves as a staging area for future stable distributions of ROS 2 and as a collection of the most recent development releases.
Unlike most stable ROS 2 distributions which have an initial release, a support window during which they are updated, and a definite end of support (see :ref:`list_of_distributions` above) the Rolling distribution is continuously updated and is subject to in-place updates which will at times include breaking changes.

Packages released into the Rolling distribution will be automatically released into future stable distributions of ROS 2.
`Releasing a ROS 2 package <Tutorials/Releasing-a-ROS-2-package-with-bloom>` into the Rolling distribution follows the same procedures as all other ROS 2 distributions.

`ROS 2 Rolling Ridley <Releases/Release-Rolling-Ridley>` is the rolling development distribution of ROS 2 as proposed in `REP 2002 <https://www.ros.org/reps/rep-2002.html>`_.
It was first introduced in June 2020.

The Rolling distribution will receive frequent and possibly compatibility-breaking releases in core packages and we recommend most people use the most recent stable distribution instead.
Since new stable distributions will be created from snapshots of the Rolling distribution, package maintainers who want to make their packages available in future ROS 2 distributions can do so by releasing their packages into the Rolling distribution.
