.. redirect-from::

    Eclipse-Oxygen-with-ROS-2-and-rviz2

Eclipse Oxygen with ROS 2 and rviz2 [community-contributed]
===========================================================

.. contents:: Table of Contents
   :depth: 1
   :local:

Setup
-----

We have installed eclipse Oxygen and git. eclipse-git (Egit) is already installed (http://www.eclipse.org/egit/download/).

We call the eclipse-workspace the same name as the ros2 package. This is not needed.

HINT: We use nested projects and so using one eclipse-workspace for one ROS-2 package, because there are many projects inside even if its one ROS-2 project, it seemed more "clean".

.. image:: https://i.imgur.com/ePQaXE3.png
   :target: https://i.imgur.com/ePQaXE3.png
   :alt: eclipse-launcher


We create a C++ Project

.. image:: https://i.imgur.com/XIsATcN.png
   :target: https://i.imgur.com/XIsATcN.png
   :alt: eclipse-1



.. image:: https://i.imgur.com/PNVxEJN.png
   :target: https://i.imgur.com/PNVxEJN.png
   :alt: eclipse-2


We choose as Project-name the name of the ROS 2 package. Makefile Project and Other Toolchain.

.. image:: https://i.imgur.com/yt5WkkN.png
   :target: https://i.imgur.com/yt5WkkN.png
   :alt: eclipse-2


Then we click on Finish

.. image:: https://i.imgur.com/Ef0tLiP.png
   :target: https://i.imgur.com/Ef0tLiP.png
   :alt: eclipse-2


In the "Project explorer" we see our Project.

.. image:: https://i.imgur.com/kYutC7W.png
   :target: https://i.imgur.com/kYutC7W.png
   :alt: eclipse-3


Inside our Project we create a folder called "src"

.. image:: https://i.imgur.com/6uFtcLT.png
   :target: https://i.imgur.com/6uFtcLT.png
   :alt: eclipse-4


Now we import a git repository

.. image:: https://i.imgur.com/pae8YOu.png
   :target: https://i.imgur.com/pae8YOu.png
   :alt: eclipse-5


We put in the repository URL

.. image:: https://i.imgur.com/HuPcPx9.png
   :target: https://i.imgur.com/HuPcPx9.png
   :alt: eclipse-6


IMPORTANT: As destination-folder for the git-repository we use the src-folder of our project we created before.

HINT: If you got problems choosing the destination folder path, the eclipse-dialog needs a name in the name field.

.. image:: https://i.imgur.com/arFZfa4.png
   :target: https://i.imgur.com/arFZfa4.png
   :alt: eclipse-7


Import using the new project wizard

.. image:: https://i.imgur.com/ety2Lxf.png
   :target: https://i.imgur.com/ety2Lxf.png
   :alt: eclipse-8


We create a General->Project

.. image:: https://i.imgur.com/rpAjqqW.png
   :target: https://i.imgur.com/rpAjqqW.png
   :alt: eclipse-9


Use as project name the same name as the git-repository. This is not needed.
IMPORTANT: Use as "Location" the folder we cloned the git repository in.

.. image:: https://i.imgur.com/nEoT0RB.png
   :target: https://i.imgur.com/nEoT0RB.png
   :alt: eclipse-10


Now we see the git-project and our project in the Project-Explorer view. We see the same files two times, but only one project is linked with Egit.

.. image:: https://i.imgur.com/sSQ8ooN.png
   :target: https://i.imgur.com/sSQ8ooN.png
   :alt: eclipse-11


We repeat this procedure again. Import git repository pluginlib

.. image:: https://i.imgur.com/hnbscVx.png
   :target: https://i.imgur.com/hnbscVx.png
   :alt: eclipse-12


IMPORTANT: As "Destination->Directory" we use a folder inside the src-folder.

.. image:: https://i.imgur.com/8Z3hlFL.png
   :target: https://i.imgur.com/8Z3hlFL.png
   :alt: eclipse-13


IMPORTANT: As location for our new project we use the folder we cloned the git repository in

.. image:: https://i.imgur.com/xySYIQi.png
   :target: https://i.imgur.com/xySYIQi.png
   :alt: eclipse-14


The same procedure again. Now with tinyxml2_vendor git repository.

.. image:: https://i.imgur.com/izC5Hke.png
   :target: https://i.imgur.com/izC5Hke.png
   :alt: eclipse-15


IMPORTANT: Again we use a folder inside the src-folder

.. image:: https://i.imgur.com/UR8S3I8.png
   :target: https://i.imgur.com/UR8S3I8.png
   :alt: eclipse-16


IMPORTANT: Use as new project folder the location of the folder we cloned.

.. image:: https://i.imgur.com/aMu1nNZ.png
   :target: https://i.imgur.com/aMu1nNZ.png
   :alt: eclipse-17


Now we see all 4 Projects in the Project-Explorer view.

.. image:: https://i.imgur.com/36zbuUx.png
   :target: https://i.imgur.com/36zbuUx.png
   :alt: eclipse-18


If we click in the top-right-corner of the Project-Explorer view we can change the Project-Presentation to Hirachical view. Now it looks like a ROS-2 project as it is on hard-drive. But this view is not good, as the linkage to Egit gets lost. So use the Flat Project-Presentation. The Egit linkage is good if you want to see e.g. which author wrote which code-line, etc.

.. image:: https://i.imgur.com/vOhRUGB.png
   :target: https://i.imgur.com/vOhRUGB.png
   :alt: eclipse-19


We go to "C/C++ build"-section and put "ament" into "Build command"

.. image:: https://i.imgur.com/vXhRwEb.png
   :target: https://i.imgur.com/vXhRwEb.png
   :alt: eclipse-26


Go to "Behavior" tab and unselect "clean" and put "build" into Build textbox.

.. image:: https://i.imgur.com/4CegjkC.png
   :target: https://i.imgur.com/4CegjkC.png
   :alt: eclipse-26


Before you can "Build Project" you need to close eclipse. Open a shell and source the ROS-2 setup.bash file, then cd into the directory of the eclipse project (here: /home/ubu/rviz2_ws/rviz2_ws) and start eclipse from inside this directory.

.. image:: https://i.imgur.com/ZyPGJLa.png
   :target: https://i.imgur.com/ZyPGJLa.png
   :alt: eclipse-27


Now you can use code-completion, egit annotations, eclipse C/C++ Tools, etc.

.. image:: https://i.imgur.com/YUEH3lM.png
   :target: https://i.imgur.com/YUEH3lM.png
   :alt: eclipse-28


Eclipse-indexer
---------------

If you open e.g. main.cpp of rviz2 you will perhaps see alot of "unresolved inclusion".You need todo the following that they disappear and that right-click->Open-Declaration will fully work. Goto Project->Properties->C++General->Path-and-Symbols and to tab References and select "ros2_ws".
IMPORTANT: If you have different eclipse-workspaces for ros2_ws and e.g. rviz2_ws, you can add your ros2_ws the same way as later the qt5 directory get added. Hint: Just add the src folder, e.g. /home/ros/ros2_ws/ros2_ws/src  not the build and install directories.


.. image:: https://i.imgur.com/mp9Pgzu.png
   :target: https://i.imgur.com/mp9Pgzu.png
   :alt: eclipse-28


Goto C/C++-General->Path-and-Symbols to tab "Source locations" and click on "Link folder". There choose the location of qt5 includes.


.. image:: https://i.imgur.com/TYgDACE.png
   :target: https://i.imgur.com/TYgDACE.png
   :alt: eclipse-28


then you see something like the next image. You could also add "excludes" (filters) to the added source locations, so that some directories dont get indexed. Its good for the "build" and "install" directories in the rviz2_ws which include duplicate headers.


.. image:: https://i.imgur.com/nv9tEAP.png
   :target: https://i.imgur.com/nv9tEAP.png
   :alt: eclipse-28


Goto C++General->Preprocessor includes, select CDT-GCC-Built-in-compiler-settings[shared] and enter into the text-box "command to get compiler specs" the following

.. code-block:: bash

   -std=c++14


.. image:: https://i.imgur.com/9DNXpDD.png
   :target: https://i.imgur.com/9DNXpDD.png
   :alt: eclipse-28


Then goto "C/C++-General->Indexer" and select the following in the image. E.g "index unused headers as c files" is to resolve e.g. QApplication, because the QApplication headers content is only "#include "qapplication.h".


.. image:: https://i.imgur.com/Wxeheak.png
   :target: https://i.imgur.com/Wxeheak.png
   :alt: eclipse-28


After running the indexer (which happens later,so you will see this also later), you can see what it added


.. image:: https://i.imgur.com/xtxZ4bg.png
   :target: https://i.imgur.com/xtxZ4bg.png
   :alt: eclipse-28


After that right-click on the rviz2 project and select "Indexer->Rebuild", after that, you see down-right a progress, you will see that it can resolve all includes.


.. image:: https://i.imgur.com/uGZaHau.png
   :target: https://i.imgur.com/uGZaHau.png
   :alt: eclipse-28


Debugging with eclipse
----------------------

Goto "C/C++-Build" and add to the build command

.. code-block:: bash

   -DCMAKE_BUILD_TYPE=Debug


.. image:: https://i.imgur.com/KXFYDHg.png
   :target: https://i.imgur.com/KXFYDHg.png
   :alt: eclipse-28


Then in eclipse goto "Run->Debug Configurations" and add the following and click on "Debug"


.. image:: https://i.imgur.com/ywzAxUP.png
   :target: https://i.imgur.com/ywzAxUP.png
   :alt: eclipse-28

