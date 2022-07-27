.. redirect-from::

    Releasing-a-ROS-2-package-with-bloom
    Guides/Releasing-a-ROS-2-package-with-bloom
    Tutorials/Releasing-a-ROS-2-package-with-bloom
    How-To-Guides/Releasing-a-ROS-2-package-with-bloom

Releasing a Package
===================

.. toctree::
   :hidden:

   First-Time-Release
   Subsequent-Releases
   Release-Team-Repository
   Release-Track
   GitHub-Manual-Authorization

**Releasing a package makes your package available on the public ROS 2 buildfarm.**
After you've created a package, this is the next step towards getting your package into the publicly available Debian packages!

Releasing your package onto the buildfarm will make your package available to be installed via package managers (eg. ``apt`` on Ubuntu) for all supported Linux platforms in a ROS distribution as described in `REP 2000 <https://ros.org/reps/rep-2000.html>`_.

Additionally, releasing the package:

#. Allows your package to have API documentation automatically generated.
#. Makes your package part of the `ROS Index <https://index.ros.org>`_.
#. (Optionally) Allows you to have automatic CI run for pull requests in your repository.

**Follow one of the guides below to get your package released:**

* :doc:`First Time Release <First-Time-Release>` - if this is the first release for the package
* :doc:`Subsequent Releases <Subsequent-Releases>` - if you are releasing a new version of a package that has already been released

After successfully following the instructions, your package will be released into the ROS ecosystem on the next distro synchronization!
