Changelog Guide
===============

* Do **not** modify the ``Forthcoming`` heading as this will be replaced with the package version number by ``catkin_prepare_release`` later on.


See `rclcpp's CHANGELOG.rst <https://github.com/ros2/rclcpp/blob/master/rclcpp/CHANGELOG.rst>`_ for a well-formatted example.
Incorrectly formatted ``CHANGELOG.rst`` can cause problems with your package.


.. warning::

   If you have any commit messages ending in an underscore, such as member variables (e.g. ``name_``) this will throw an error with the RST Changelog format because RST treats those as `link targets <http://docutils.sourceforge.net/docs/user/rst/quickstart.html#sections>`_.
   The error will be something like:

   .. code-block::

      <string>:21: (ERROR/3) Unknown target name: "name".

   To fix this, you'll need to escape the variable, for example:

   .. code-block::

      * fix for checking the ``name_``
