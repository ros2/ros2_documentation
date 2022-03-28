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
   Release-Preparation
   Release
   Releasing-a-Third-Party-Package
   Github-Manual-Authorization

Releasing a package in ROS2 means to make your package available on the public ROS 2 buildfarm.
After you've created a package, this is the next step towards getting your package in to the
publicly-available Debian packages so that others can install the package via ``apt``.

**Select and follow the instructions** for the type of package you are releasing:

* `Releasing an Ament Package`_

   A package which contains a ``package.xml`` and follows the packaging guidelines
   of ament, regardless of the underlying build system. Most ROS packages are ament packages.
   
* `Releasing a Third Party Package`_

   A third party package that doesn't follow the packaging guidelines of ament and doesn't contain
   a ``package.xml``.

Releasing an Ament Package
--------------------------

An ament package is any package which contains a package.xml and follows the packaging guidelines
of ament, regardless of the underlying build system. Most ROS packages are ament packages.

Conduct the following in order:

#. :doc:`Obtain Access to Release Repository <Obtain-Access-to-Release-Repository>`
#. :doc:`Release Preparation <Release-Preparation>`
#. :doc:`Release <Release>`

Releasing a Third Party Package
-------------------------------

A third party package is a software package which exists outside of the ROS
ecosystem, is used by packages in the ROS ecosystem, but is not released widely as a system
dependency.
These software packages might be designed to be used outside of the ROS ecosystem,
but are not big enough or mature enough to be released into operating system package managers.
Instead they are released using the ROS infrastructure along side a ROS distribution as if
they were actually ROS packages.

They require a separate release procedure as they don't follow the packaging guidelines of
ament and don't contain a ``package.xml``.

Conduct the following in order:

#. :doc:`Obtain Access to Release Repository <Obtain-Access-to-Release-Repository>`
#. :doc:`Releasing a Third Party Package <Releasing-a-Third-Party-Package>`
