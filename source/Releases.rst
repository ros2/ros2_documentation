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

   Releases/Release-Jazzy-Jalisco
   Releases/Release-Humble-Hawksbill
   Releases/Release-Rolling-Ridley
   Releases/Development
   Releases/End-of-Life
   Releases/Release-Process

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
     .rst-content table.docutils:not(.field-list) tr:nth-child(3) td {background-color: #33cc66;}
     .rst-content tr:nth-child(3) {background-color: #33cc66;}
   </style>

.. |rolling| image:: Releases/rolling-small.png
   :alt: Rolling logo

.. |jazzy| image:: Releases/jazzy-small.png
   :alt: Jazzy logo

.. |iron| image:: Releases/iron-small.png
   :alt: Iron logo

.. |humble| image:: Releases/humble-small.png
   :alt: Humble logo

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
   :widths: 35 25 30 20 10

   * - Distro
     - Release date
     - Logo
     - EOL date
     - ROS Boss
   * - :doc:`Jazzy Jalisco <Releases/Release-Jazzy-Jalisco>`
     - May 23, 2024
     - |jazzy|
     - May 2029
     - `Marco A. Gutiérrez <https://github.com/marcoag>`_
   * - :doc:`Iron Irwini <Releases/Release-Iron-Irwini>`
     - May 23, 2023
     - |iron|
     - December 4, 2024
     - `Yadunund Vijay <https://github.com/Yadunund>`_
   * - :doc:`Humble Hawksbill <Releases/Release-Humble-Hawksbill>`
     - May 23, 2022
     - |humble|
     - May 2027
     - `Audrow Nash <https://github.com/audrow>`_
   * - :doc:`Galactic Geochelone <Releases/Release-Galactic-Geochelone>`
     - May 23, 2021
     - |galactic|
     - December 9, 2022
     - `Scott Logan <https://github.com/cottsay/>`_
   * - :doc:`Foxy Fitzroy <Releases/Release-Foxy-Fitzroy>`
     - June 5, 2020
     - |foxy|
     - June 20, 2023
     - `Jacob Perron <https://github.com/jacobperron>`_ / `Dharini Dutia <https://github.com/quarkytale>`_
   * - :doc:`Eloquent Elusor <Releases/Release-Eloquent-Elusor>`
     - November 22, 2019
     - |eloquent|
     - November 2020
     - `Michael Carroll <https://github.com/mjcarroll>`_
   * - :doc:`Dashing Diademata <Releases/Release-Dashing-Diademata>`
     - May 31, 2019
     - |dashing|
     - May 2021
     - `Steven! Ragnarök <https://github.com/nuclearsandwich>`_
   * - :doc:`Crystal Clemmys <Releases/Release-Crystal-Clemmys>`
     - December 14, 2018
     - |crystal|
     - December 2019
     - `Steven! Ragnarök <https://github.com/nuclearsandwich>`_
   * - :doc:`Bouncy Bolson <Releases/Release-Bouncy-Bolson>`
     - July 2, 2018
     - |bouncy|
     - July 2019
     - `Mikael Arguedas <https://github.com/mikaelarguedas>`_ / `Steven! Ragnarök <https://github.com/nuclearsandwich>`_
   * - :doc:`Ardent Apalone <Releases/Release-Ardent-Apalone>`
     - December 8, 2017
     - |ardent|
     - December 2018
     - `Steven! Ragnarök <https://github.com/nuclearsandwich>`_
   * - :doc:`beta3 <Releases/Beta3-Overview>`
     - September 13, 2017
     -
     - December 2017
     -
   * - :doc:`beta2 <Releases/Beta2-Overview>`
     - July 5, 2017
     -
     - September 2017
     -
   * - :doc:`beta1 <Releases/Beta1-Overview>`
     - December 19, 2016
     -
     - Jul 2017
     -
   * - :doc:`alpha1 - alpha8 <Releases/Alpha-Overview>`
     - August 31, 2015
     -
     - December 2016
     -

Future Distributions
--------------------

For details on upcoming features see the :doc:`roadmap <The-ROS2-Project/Roadmap>`.

There is a new ROS 2 distribution released yearly on May 23rd (`World Turtle Day <https://www.worldturtleday.org/>`_).

.. list-table::
   :class: future-distros
   :header-rows: 1
   :widths: 35 30 20 15

   * - Distro
     - Release date
     - Logo
     - EOL date
   * - :doc:`Kilted Kaiju <Releases/Release-Kilted-Kaiju>`
     - May 2025
     - TBD
     - Nov 2026


.. _rolling_distribution:

Rolling Distribution
--------------------

:doc:`ROS 2 Rolling Ridley <Releases/Release-Rolling-Ridley>` is the rolling development distribution of ROS 2.
It is described in `REP 2002 <https://www.ros.org/reps/rep-2002.html>`_ and was first introduced in June 2020.

The Rolling distribution of ROS 2 serves two purposes:

1. it is a staging area for future stable distributions of ROS 2, and
2. it is a collection of the most recent development releases.

As the name implies, Rolling is continuously updated and **can have in-place updates that include breaking changes**.
We recommend that most people use the most recent stable distribution instead (see :ref:`list_of_distributions`).

Packages released into the Rolling distribution will be automatically released into future stable distributions of ROS 2.
:doc:`Releasing a ROS 2 package <../How-To-Guides/Releasing/Releasing-a-Package>` into the Rolling distribution follows the same procedures as all other ROS 2 distributions.
