.. _MigratingDocsFromROS1Wiki:

Migrating docs from ROS 1 wiki
==================================

.. parsed-literal::

    Area: Contributing | Content-type: how-to | Experience: beginner

The first step in migrating a page from the `ROS Wiki <https://wiki.ros.org>`_ to the ROS 2 documentation is to determine if the page needs to be migrated.
Check if the content, or something similar, is available on https://docs.ros.org/en/rolling by searching for related terms.
If it has already been migrated, congratulations!
You are done.

If it hasn't been migrated, then consider whether it is worth keeping.
Pages that you or others find useful, and refer to regularly, are good candidates assuming they have not been superseded by other documentation.
Pages for ROS projects and features that are no longer supported by a current distribution should not be migrated.

The next step for migrating a ROS Wiki page is to determine the correct location for the migrated page.
Only ROS Wiki pages that cover core ROS concepts belong in the ROS Documentation, these pages should be migrated to a logical location within the ROS documentation.
Package specific documentation should be migrated to the package-level documentation generated in the package's source repository.
Once the package level documentation has been updated it will be visible `as part of the package-level documentation <https://docs.ros.org/en/rolling/p/>`__.
If you are unsure whether and where to migrate a page, please get in touch via an issue on https://github.com/ros2/ros2_documentation or on https://discourse.ros.org.

Once you've determined that a ROS Wiki page is worth migrating, and found an appropriate landing spot in the ROS documentation, the next step in the migration process is to set up the conversion tools necessary to migrate the page.
In most cases the only tools necessary to migrate a single ROS Wiki page to the ROS Docs are the `PanDoc <https://pandoc.org/>`_ command line tool and a text editor.
PanDoc is supported by most modern operating systems using the installation instruction found on their website.
It is worth noting that the ROS Wiki uses an older wiki technology (MoinMoin), so the markup language used is an obscure dialect of the `MediaWiki <https://www.mediawiki.org/wiki/Help:Formatting>`__ format.
We've found that the easiest way to migrate a page from the ROS Wiki is to convert it from HTML into reStructured text using PanDoc.


Migrating a Wiki File
---------------------

#. Clone the appropriate repository.
   If you are migrating a page to the official documentation hosted here, then you should clone https://github.com/ros2/ros2_documentation.

#. Create a new Github branch for your migrated page.
   We suggest something like ``pagename-migration``.

#. Download the appropriate ROS Wiki page to an html file using wget or a similar tool (e.g. ``wget -O urdf.html https://wiki.ros.org/urdf``).
   Alternatively you can use your web browser to save the page's HTML.

#. Next you need to remove the extraneous HTML in the file you downloaded
   Using your browser's developer mode, find the name of the first useful HTML element in the Wiki page.
   In most cases all of the HTML between the third line of the file, starting with the ``<head>`` tag, through the start of the first ``<h1>`` tag can be safely removed.
   In the case where there is a table of contents, the first useful tag may be an ``<h2>`` tag.
   Similarly, the ROS wiki contains some footer text that starts with ``<div id="pagebottom"></div>`` and ends just above ``</body></html>`` that can also be removed.

#. Convert your html file by running a PanDoc conversion between HTML and restructured text.
   The following command converts an HTML file to the equivalent reStructured text files: ``pandoc -f html -t rst urdf.html > URDF.rst``.

#. Attempt to build your new documentation using the ``make html`` command.
   There may be errors and warnings that you will need to address.

#. **CAREFULLY** read through the entire page making sure the material is up to date for ROS 2.
   Check every single link to make sure it points to the appropriate location on docs.ros.org.
   Internal document references must be updated to point to the equivalent ROS 2 material.
   Your updated document should not point to the ROS Wiki unless it is absolutely necessary.
   This process may require you alter the document considerably, and you may need to pull multiple wiki files.
   You should verify that every code sample in the document is working correctly under ROS 2.

#. Find and download any images that may be in the old document.
   The easiest way to do this is to right click in the browser and download all of the images.
   Alternatively you can find images by searching for ``<img src>`` tags in the HTML file.

#. For each image files downloaded update the image file links to point to the correct image directory for the ROS Docs.
   If any of the images require updating, or could be replaced with a `Mermaid <https://mermaid.js.org/intro/>`__ chart, please make this change.
   Be aware that Mermaid.js is only supported in the core ROS 2 documentation currently.

#. Once your document is complete add a table of contents to the top of your new rst document using the appropriate Sphinx commands.
   This block should replace any existing table of contents from the old ROS Wiki.

#. Issue your pull request.
   Make sure to point to the original ROS Wiki file for reference.

#. Once your pull request has been accepted please add a note to the top of the page on the original ROS Wiki article pointing to the new documentation page.

For a real-world example of this process in action, please refer to the ROS 2 Image Processing Pipeline in both `the ROS 2 Docs <https://github.com/ros-perception/image_pipeline/blob/rolling/image_pipeline/doc/tutorials.rst>`__ and in the original `ROS Wiki <https://wiki.ros.org/image_pipeline>`__.
The completed documentation page can be found in the `ROS 2 package documentation for image_pipeline <https://docs.ros.org/en/rolling/p/image_pipeline/>`__.
