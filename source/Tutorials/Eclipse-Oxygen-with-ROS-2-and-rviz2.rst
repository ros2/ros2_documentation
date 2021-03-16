.. redirect-from::

    Eclipse-Oxygen-with-ROS-2-and-rviz2

Eclipse Oxygen with ROS 2 and rviz2 [community-contributed]
===========================================================

.. contents:: Table of Contents
   :depth: 1
   :local:

Setup
-----

This tutorial assumes Eclipse Oxygen, git, and Egit (http://www.eclipse.org/egit/download/) are already installed

Throughout the tutorial we name the eclipse workspace the same name as the ros2 package, but this is not required.

HINT: We use nested projects and one Eclipse Workspace for each ROS-2 package.

.. image:: https://i.imgur.com/ePQaXE3.png
   :target: https://i.imgur.com/ePQaXE3.png
   :alt: eclipse-launcher


Create a C++ Project.

.. image:: https://i.imgur.com/XIsATcN.png
   :target: https://i.imgur.com/XIsATcN.png
   :alt: eclipse-1



.. image:: https://i.imgur.com/PNVxEJN.png
   :target: https://i.imgur.com/PNVxEJN.png
   :alt: eclipse-2


Choose the ROS 2 package name as the Project Name.
Choose a Makefile Project and Other Toolchain.

.. image:: https://i.imgur.com/yt5WkkN.png
   :target: https://i.imgur.com/yt5WkkN.png
   :alt: eclipse-2


Click on Finish

.. image:: https://i.imgur.com/Ef0tLiP.png
   :target: https://i.imgur.com/Ef0tLiP.png
   :alt: eclipse-2


Our project should be shown in the "Project Explorer".

.. image:: https://i.imgur.com/kYutC7W.png
   :target: https://i.imgur.com/kYutC7W.png
   :alt: eclipse-3


Inside our Project create a folder called "src".

.. image:: https://i.imgur.com/6uFtcLT.png
   :target: https://i.imgur.com/6uFtcLT.png
   :alt: eclipse-4


Import a git repository.

.. image:: https://i.imgur.com/pae8YOu.png
   :target: https://i.imgur.com/pae8YOu.png
   :alt: eclipse-5


Put in the repository URL.

.. image:: https://i.imgur.com/HuPcPx9.png
   :target: https://i.imgur.com/HuPcPx9.png
   :alt: eclipse-6


IMPORTANT: Use the source folder of the project we created before as the destination folder.

HINT: If you ran into problems choosing the destination folder path, the Eclipse Dialog needs a name in the name field.

.. image:: https://i.imgur.com/arFZfa4.png
   :target: https://i.imgur.com/arFZfa4.png
   :alt: eclipse-7


Import using the new project wizard.

.. image:: https://i.imgur.com/ety2Lxf.png
   :target: https://i.imgur.com/ety2Lxf.png
   :alt: eclipse-8


Create a General->Project.

.. image:: https://i.imgur.com/rpAjqqW.png
   :target: https://i.imgur.com/rpAjqqW.png
   :alt: eclipse-9


Use the git repository name as the project name.
IMPORTANT: Use the folder we cloned the git repository in as the "Location".

.. image:: https://i.imgur.com/nEoT0RB.png
   :target: https://i.imgur.com/nEoT0RB.png
   :alt: eclipse-10


The git project and the new project should be visible in the Project Explorer view.
The same files are listed multiple times, but only one project is linked with Egit.

.. image:: https://i.imgur.com/sSQ8ooN.png
   :target: https://i.imgur.com/sSQ8ooN.png
   :alt: eclipse-11


Repeat this procedure again.
Import git repository pluginlib.

.. image:: https://i.imgur.com/hnbscVx.png
   :target: https://i.imgur.com/hnbscVx.png
   :alt: eclipse-12


IMPORTANT: Use a folder inside the source folder as "Destination->Directory".

.. image:: https://i.imgur.com/8Z3hlFL.png
   :target: https://i.imgur.com/8Z3hlFL.png
   :alt: eclipse-13


IMPORTANT: Use the folder we cloned the git repository in as the location for the new project.

.. image:: https://i.imgur.com/xySYIQi.png
   :target: https://i.imgur.com/xySYIQi.png
   :alt: eclipse-14


Run the same procedure with the tinyxml2_vendor git repository.

.. image:: https://i.imgur.com/izC5Hke.png
   :target: https://i.imgur.com/izC5Hke.png
   :alt: eclipse-15


IMPORTANT: Again use a folder inside the source folder.

.. image:: https://i.imgur.com/UR8S3I8.png
   :target: https://i.imgur.com/UR8S3I8.png
   :alt: eclipse-16


IMPORTANT: Use the location of the folder we cloned as the new project folder.

.. image:: https://i.imgur.com/aMu1nNZ.png
   :target: https://i.imgur.com/aMu1nNZ.png
   :alt: eclipse-17


Now all four Projects should be visible in the Project Explorer view.

.. image:: https://i.imgur.com/36zbuUx.png
   :target: https://i.imgur.com/36zbuUx.png
   :alt: eclipse-18


Clicking in the top right cornder for the Project Explorer view allows us to change the Project Presentation to Hierarchical view.
Now it looks like a ROS-2 project as it is on the hard drive.
But this view loses the linkage to Egit, so use the Flat Project Presentation.
The Egit linkage is good if you want to see e.g. which author wrote which code-line, etc.

.. image:: https://i.imgur.com/vOhRUGB.png
   :target: https://i.imgur.com/vOhRUGB.png
   :alt: eclipse-19


Go to "C/C++ build"-section and put "ament" into "Build command".

.. image:: https://i.imgur.com/vXhRwEb.png
   :target: https://i.imgur.com/vXhRwEb.png
   :alt: eclipse-26


Go to "Behavior" tab and unselect "clean" and put "build" into Build textbox.

.. image:: https://i.imgur.com/4CegjkC.png
   :target: https://i.imgur.com/4CegjkC.png
   :alt: eclipse-26


Before "Build project" will work, we need to close Eclipse.
Open a shell and source the ROS-2 setup.bash file, then cd into the directory of the eclipse project (here: /home/ubu/rviz2_ws/rviz2_ws) and start Eclipse from inside this directory.

.. image:: https://i.imgur.com/ZyPGJLa.png
   :target: https://i.imgur.com/ZyPGJLa.png
   :alt: eclipse-27


Now code completion, egit annotations, eclipse C/C++ Tools, etc. should all work.

.. image:: https://i.imgur.com/YUEH3lM.png
   :target: https://i.imgur.com/YUEH3lM.png
   :alt: eclipse-28


Eclipse-indexer
---------------

Opening the main.cpp of rviz2 may show a lot of "unresolved inclusion" warnings.
To fix this, go to Project->Properties->C++ General->Path and Symbols.
Click on the "References" tab and select "ros2_ws".


.. image:: https://i.imgur.com/mp9Pgzu.png
   :target: https://i.imgur.com/mp9Pgzu.png
   :alt: eclipse-28


Go to C/C++-General->Path-and-Symbols, click on the "Source locations" tab and click on "Link folder".
Choose the location of qt5 includes.


.. image:: https://i.imgur.com/TYgDACE.png
   :target: https://i.imgur.com/TYgDACE.png
   :alt: eclipse-28


The next image should be shown.
It is a good idea to add excludes to the source locations, so that some directories (like "Build" and "Install") don't get indexed.


.. image:: https://i.imgur.com/nv9tEAP.png
   :target: https://i.imgur.com/nv9tEAP.png
   :alt: eclipse-28


Go to C++General->Preprocessor includes, select "CDT GCC Built in compiler settings [Shared]" and enter in the "command to get compiler specs" text box the following:

.. code-block:: bash

   -std=c++14


.. image:: https://i.imgur.com/9DNXpDD.png
   :target: https://i.imgur.com/9DNXpDD.png
   :alt: eclipse-28


Go to "C/C++-General->Indexer" and select the following in the image.
E.g "index unused headers as c files" to resolve e.g. QApplication, because the QApplication headers content is only "#include "qapplication.h".


.. image:: https://i.imgur.com/Wxeheak.png
   :target: https://i.imgur.com/Wxeheak.png
   :alt: eclipse-28


After running the indexer (which happens later, so you will see this also later), you can see what it added


.. image:: https://i.imgur.com/xtxZ4bg.png
   :target: https://i.imgur.com/xtxZ4bg.png
   :alt: eclipse-28


After that right-click on the rviz2 project and select "Indexer->Rebuild", which will start rebuilding the index (there is an icon in the lower right showing progress).
Once the index is finished rebuilding, it should be able to resolve all includes.


.. image:: https://i.imgur.com/uGZaHau.png
   :target: https://i.imgur.com/uGZaHau.png
   :alt: eclipse-28


Debugging with eclipse
----------------------

Go to "C/C++-Build" and add to the build command:

.. code-block:: bash

   -DCMAKE_BUILD_TYPE=Debug


.. image:: https://i.imgur.com/KXFYDHg.png
   :target: https://i.imgur.com/KXFYDHg.png
   :alt: eclipse-28


Then in eclipse go to "Run->Debug Configurations" and add the following and click on "Debug".


.. image:: https://i.imgur.com/ywzAxUP.png
   :target: https://i.imgur.com/ywzAxUP.png
   :alt: eclipse-28
