Every release of the package must have a unique version number higher than the previous release.
Run:

.. code-block:: bash

   catkin_prepare_release

which performs the following:

#. increases the package version in ``package.xml``
#. replaces the heading ``Forthcoming`` with ``version (date)`` (eg. ``0.0.1 (2022-01-08)``) in ``CHANGELOG.rst``
#. commits those changes
#. creates a tag (eg. ``0.0.1``)
#. pushes the changes and the tag to your remote repository

.. note::

   By default the patch version of the package is incremented, such as from ``0.0.0`` to ``0.0.1``.
   To increment the minor or major version instead, run ``catkin_prepare_release --bump minor`` or ``catkin_prepare_release --bump major``.
   For more details, see ``catkin_prepare_release --help``.
