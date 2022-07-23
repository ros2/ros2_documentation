Every release of the package must have a unique version number.
Run:

.. code-block:: bash

   catkin_prepare_release

which performs the following:

#. increases the package version in ``package.xml``
#. replaces the heading ``Forthcoming`` with ``version (date)`` (eg. ``0.0.1 (2022-01-08)``) in ``CHANGELOG.rst``
#. commits those changes
#. creates a tag (eg. ``0.0.1``)
#. pushes those changes to upstream

.. note::

   By default this command increases the patch version of your package, e.g. ``0.0.0`` -> ``0.0.1``, but you can pick minor or major using the ``--bump`` option.
