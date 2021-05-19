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
   Releases/Release-Rolling-Ridley.rst

.. |galactic| image:: Releases/galactic-small.png
.. |foxy| image:: Releases/foxy-small.png
.. |eloquent| image:: Releases/eloquent-small.png
.. |dashing| image:: Releases/dashing-small.png
.. |crystal| image:: Releases/crystal-small.png
.. |bouncy| image:: Releases/bouncy-small.png
.. |ardent| image:: Releases/ardent-small.png

.. raw:: html

    <!--
    The CSS and HTML below generate the list of current and historic ROS 2 distributions.
    It is currently using raw HTML because there was no way that I could see to make the
    ReStructured Text "list-tables" directive work with Read-the-docs.
    -->
    <style>
    table.distroclass, th.distroclass {
      border: 1px solid #e1e4e5;
      border-top-color: rgb(225, 228, 229);
      border-top-style: solid;
      border-top-width: 1px;
      border-right-color: rgb(225, 228, 229);
      border-right-style: solid;
      border-right-width: 1px;
      border-bottom-color: rgb(225, 228, 229);
      border-bottom-style: solid;
      border-bottom-width: 1px;
      border-left-color: rgb(225, 228, 229);
      border-left-style: solid;
      border-left-width: 1px;
      border-image-outset: 0;
      border-image-repeat: stretch;
      border-image-slice: 100%;
      border-image-source: none;
      border-image-width: 1;
    }
    td.distroclass {
      border-top-color: rgb(225, 228, 229);
      border-top-style: solid;
      border-top-width: 1px;
      border-right-color: rgb(225, 228, 229);
      border-right-style: solid;
      border-right-width: 1px;
      border-bottom-color: rgb(225, 228, 229);
      border-bottom-style: solid;
      border-bottom-width: 1px;
      border-left-color: rgb(225, 228, 229);
      border-left-style: solid;
      border-left-width: 1px;
      vertical-align: middle;
      padding-left: 15px;
    }
    thead.distroclass {
      color: #000;
      text-align: left;
      vertical-align: bottom;
      white-space: nowrap;
    }
    tr.distroclass:nth-child(2n-1) {
      background-color: #f3f6f6;
    }
    tr.active {
      background-color: #33cc66;
    }
    </style>
    <table class="distroclass">
      <colgroup>
        <col style="width: 35%" />
        <col style="width: 30%" />
        <col style="width: 20%" />
        <col style="width: 15%" />
      </colgroup>
      <thead>
        <tr>
          <th class="distroclass"><p>Distro</p></th>
          <th class="distroclass"><p>Release date</p></th>
          <th class="distroclass"><p>Logo</p></th>
          <th class="distroclass"><p>EOL Date</p></th>
        </tr>
      </thead>
      <tbody>
        <tr class="active">
          <td class="distroclass"><p><a class="reference internal" href="Releases/Release-Galactic-Geochelone.html">Galactic Geochelone</a></p></td>
          <td class="distroclass"><p>May 23, 2021</p></td>
          <td class="distroclass"><p><img alt="Galactic logo" src="_images/galactic-small.png" /></p></td>
          <td class="distroclass"><p>November 2022</p></td>
        </tr>
        <tr class="active">
          <td class="distroclass"><p><a class="reference internal" href="Releases/Release-Foxy-Fitzroy.html">Foxy Fitzroy</a></p></td>
          <td class="distroclass"><p>June 5, 2020</p></td>
          <td class="distroclass"><p><img alt="Foxy logo" src="_images/foxy-small.png" /></p></td>
          <td class="distroclass"><p>May 2023</p></td>
        </tr>
        <tr class="distroclass">
          <td class="distroclass"><p><a class="reference internal" href="Releases/Release-Eloquent-Elusor.html">Eloquent Elusor</a></p></td>
          <td class="distroclass"><p>November 22nd, 2019</p></td>
          <td class="distroclass"><p><img alt="Eloquent logo" src="_images/eloquent-small.png" /></p></td>
          <td class="distroclass"><p>November 2020</p></td>
        </tr>
        <tr class="active">
          <td class="distroclass"><p><a class="reference internal" href="Releases/Release-Dashing-Diademata.html">Dashing Diademata</a></p></td>
          <td class="distroclass"><p>May 31st, 2019</p></td>
          <td class="distroclass"><p><img alt="Dashing logo" src="_images/dashing-small.png" /></p></td>
          <td class="distroclass"><p>May 2021</p></td>
        </tr>
        <tr class="distroclass">
          <td class="distroclass"><p><a class="reference internal" href="Releases/Release-Crystal-Clemmys.html">Crystal Clemmys</a></p></td>
          <td class="distroclass"><p>December 14th, 2018</p></td>
          <td class="distroclass"><p><img alt="Crystal logo" src="_images/crystal-small.png" /></p></td>
          <td class="distroclass"><p>December 2019</p></td>
        </tr>
        <tr class="distroclass">
          <td class="distroclass"><p><a class="reference internal" href="Releases/Release-Bouncy-Bolson.html">Bouncy Bolson</a></p></td>
          <td class="distroclass"><p>July 2nd, 2018</p></td>
          <td class="distroclass"><p><img alt="Bouncy logo" src="_images/bouncy-small.png" /></p></td>
          <td class="distroclass"><p>July 2019</p></td>
        </tr>
        <tr class="distroclass">
          <td class="distroclass"><p><a class="reference internal" href="Releases/Release-Ardent-Apalone.html">Ardent Apalone</a></p></td>
          <td class="distroclass"><p>December 8th, 2017</p></td>
          <td class="distroclass"><p><img alt="Ardent logo" src="_images/ardent-small.png" /></p></td>
          <td class="distroclass"><p>December 2018</p></td>
        </tr>
        <tr class="distroclass">
          <td class="distroclass"><p><a class="reference internal" href="Releases/Beta3-Overview.html">beta3</a></p></td>
          <td class="distroclass"><p>September 13th, 2017</p></td>
          <td class="distroclass"/>
          <td class="distroclass"><p>December 2017</p></td>
        </tr>
        <tr class="distroclass">
          <td class="distroclass"><p><a class="reference internal" href="Releases/Beta2-Overview.html">beta2</a></p></td>
          <td class="distroclass"><p>July 5th, 2017</p></td>
          <td class="distroclass"/>
          <td class="distroclass"><p>September 2017</p></td>
        </tr>
        <tr class="distroclass">
          <td class="distroclass"><p><a class="reference internal" href="Releases/Beta1-Overview.html">beta1</a></p></td>
          <td class="distroclass"><p>December 19th, 2016</p></td>
          <td class="distroclass"/>
          <td class="distroclass"><p>July 2017</p></td>
        </tr>
        <tr class="distroclass">
          <td class="distroclass"><p><a class="reference internal" href="Releases/Alpha-Overview.html">alpha1 - alpha8</a></p></td>
          <td class="distroclass"><p>August 31st, 2015</p></td>
          <td class="distroclass"/>
          <td class="distroclass"><p>December 2016</p></td>
        </tr>
      </tbody>
    </table>
    <br/>

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
`Releasing a ROS 2 package <Guides/Releasing-a-ROS-2-package-with-bloom>` into the Rolling distribution follows the same procedures as all other ROS 2 distributions.

`ROS 2 Rolling Ridley <Releases/Release-Rolling-Ridley>` is the rolling development distribution of ROS 2 as proposed in `REP 2002 <https://www.ros.org/reps/rep-2002.html>`_.
It was first introduced in June 2020.

The Rolling distribution will receive frequent and possibly compatibility-breaking releases in core packages and we recommend most people use the most recent stable distribution instead.
Since new stable distributions will be created from snapshots of the Rolling distribution, package maintainers who want to make their packages available in future ROS 2 distributions can do so by releasing their packages into the Rolling distribution.
