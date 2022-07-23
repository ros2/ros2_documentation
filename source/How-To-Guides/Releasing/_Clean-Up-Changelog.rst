Open ``CHANGELOG.rst`` in an editor.
You will see that ``catkin_generate_changelog`` has populated the file with commit messages, like below:

.. code-block:: rst

   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   Changelog for package your_package
   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

   Forthcoming
   -----------
   * you can modify commit message here
   * and here

Clean up the list of commit messages to concisely convey  to your users and maintainers, the notable changes have been made since the last release.

**Commit the ``CHANGELOG.rst`` files you cleaned up.**

See `rclcpp's CHANGELOG.rst <https://github.com/ros2/rclcpp/blob/master/rclcpp/CHANGELOG.rst>`_ for a well-formatted example.
Incorrectly formatted ``CHANGELOG.rst`` can cause problems with your package.

.. note::

   You should **not** modify the ``Forthcoming`` heading, as this will be replaced with the package version number by ``catkin_prepare_release`` later on.

.. warning::

   If you have any commit messages ending in an underscore, such as member variables (e.g. ``name_``) this will throw an error with the RST Changelog format because RST treats those as `link targets <http://docutils.sourceforge.net/docs/user/rst/quickstart.html#sections>`_.
   The error will be something like:

   .. code-block::

      <string>:21: (ERROR/3) Unknown target name: "name".

   To fix this, you'll need to escape the variable, for example:

   .. code-block::

      * fix for checking the ``name_``
