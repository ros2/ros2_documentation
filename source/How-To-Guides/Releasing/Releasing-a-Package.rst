.. redirect-from::

    Releasing-a-ROS-2-package-with-bloom
    Guides/Releasing-a-ROS-2-package-with-bloom
    Tutorials/Releasing-a-ROS-2-package-with-bloom
    How-To-Guides/Releasing-a-ROS-2-package-with-bloom

Releasing a Package
===================

.. toctree::
   :hidden:

   Request-Access-to-Release-Repository
   First-Time-Release
   Successive-Releases
   Releasing-a-Third-Party-Package
   GitHub-Manual-Authorization
   Changelog-Guide
   Release-Track-Settings

Releasing a package in ROS 2 means to make your package available on the public ROS 2 buildfarm.
After you've created a package, this is the next step towards getting your package in to the publicly-available Debian packages so that others can install the package via ``apt``.

First, **follow instructions to** :doc:`Request Access to Release Repository <Request-Access-to-Release-Repository>`.

Second, **select one of the following**, according to the type of package you are releasing:

* :doc:`First Time Release <First-Time-Release>`

   If you are ready to release a ROS package that you have developed for the first time, follow this guide.

* :doc:`Successive Releases <Successive-Releases>`

   If you have already released a ROS package, you can release a new version of it with this guide.

* :doc:`Releasing a Third Party Package <Releasing-a-Third-Party-Package>`

   A third party package that doesn't follow the packaging guidelines of ament and doesn't contain a ``package.xml``.

Once you successfully follow the instructions, your package will be released into the ROS ecosystem on the next synchronization!
