.. redirect-from::

    Inter-Sphinx-Support

Using Sphinx for cross-referencing packages
===========================================

.. contents:: Table of Contents
   :depth: 1
   :local:

This page provides a quick guide on how you can cross-reference package documentation within rosindex using Sphinx.

Inventory files must be added to ROSIndex Sphinx's ``conf.py`` file found `here <https://github.com/ros2/rosindex/blob/ros2/_sphinx/conf.py>`__. Note that the ``URI`` added to the configuration file must point to the directory where the ``.inv`` file is rather than to the file itself (i.e: ``https://docs.ros.org/independent/api/catkin_pkg`` instead of ``https://docs.ros.org/independent/api/catkin_pkg/objects.inv``).


Showing all links of an Intersphinx mapping file
------------------------------------------------

(Partially borrowed from `here <http://www.sphinx-doc.org/en/master/usage/extensions/intersphinx.html>`__).

To show all Intersphinx links and their targets of an Intersphinx mapping file, either local or remote, run:

.. code-block:: bash

    python -msphinx.ext.intersphinx "url-or-path-to-inv-file"

This is helpful when searching for the root cause of a broken Intersphinx link in a documentation project.


Linking to other sites using Intersphinx
----------------------------------------

(Partially borrowed from `here <https://my-favorite-documentation-test.readthedocs.io/en/latest/using_intersphinx.html>`__).

* You may supply an explicit title and reference target: ``:role:\`title <target>\``` will refer to target, but the link text will be title.
* If you prefix the content with !, no reference/hyperlink will be created.
* If you prefix the content with ~, the link text will only be the last component of the target. For example, ``:py:meth:\`~Queue.Queue.get\``` will refer to ``Queue.Queue.get`` but only display get as the link text.


Examples of intersphinx in action
---------------------------------

Links to **source code** can be created as follows:

.. note::

    Class :class:`vcstools.VcsClient` implements the :meth:`vcstools.VcsClient.checkout` method.

Class :class:`vcstools.VcsClient` implements the :meth:`vcstools.VcsClient.checkout` method.

------------

Links to documentation pages:

.. note::

    Refer to :doc:`vcstools Developer's Guide document<developers_guide>`.

Refer to :doc:`vcstools Developer's Guide document<developers_guide>`.

------------

Links to other pages in this documentation:

.. note::

    See `the installation page <../Installation>`.

See `the installation page <../Installation>`.
