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

   By default this command increases the patch version of your package, e.g. ``0.1.1`` -> ``0.1.2``, but you can pick minor or major using the ``--bump`` option.

.. note::

   Even if you do not use ``catkin_prepare_release``, you must have one or more valid ``package.xml`` with the same version and a matching tag in your upstream repository.
   For example, if you are going to release version 0.1.0 of your package, then bloom expects there to be a 0.1.0 tag in your upstream repository.

   If you have a custom version tagging scheme you'd like to use, then bloom can handle while configuring a release track using the 'Release Tag' configuration.
