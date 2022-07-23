.. redirect-from::

    Releasing-a-ROS-2-package-with-bloom
    Guides/Releasing-a-ROS-2-package-with-bloom
    Tutorials/Releasing-a-ROS-2-package-with-bloom
    How-To-Guides/Releasing-a-ROS-2-package-with-bloom

Releasing a Package
===================

.. toctree::
   :hidden:

   Obtain-Access-to-Release-Repository
   First-Time-Release-of-a-Catkin-Package
   Successive-Releases-for-a-Catkin-Package
   Releasing-a-Third-Party-Package
   Github-Manual-Authorization
   Release-Track-Settings

Releasing a package in ROS 2 means to make your package available on the public ROS 2 buildfarm.
After you've created a package, this is the next step towards getting your package in to the publicly-available Debian packages so that others can install the package via ``apt``.

First, **follow instructions to** :doc:`Obtain Access to Release Repository <Obtain-Access-to-Release-Repository>`.

Second, **select one of the following**, according to the type of package you are releasing:

* :doc:`First Time Release of a Catkin Package <First-Time-Release-of-a-Catkin-Package>`

   If you have developed a ROS package and are ready to release it for the first time, follow this guide.

* :doc:`Successive Releases for a Catkin Package <Successive-Releases-for-a-Catkin-Package>`

   If you are a package maintainer and have some new changes you want to release, follow this guide.

* :doc:`Releasing a Third Party Package <Releasing-a-Third-Party-Package>`

   A third party package that doesn't follow the packaging guidelines of ament and doesn't contain a ``package.xml``.

Once you successfully follow the instructions, your package should be released into the ROS ecosystem on the next synchronization!
