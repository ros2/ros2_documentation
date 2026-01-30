diff --git a/source/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.rst b/source/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.rst
index 65ac7e8..abcd123 100644
--- a/source/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.rst
+++ b/source/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.rst
@@ -360,7 +360,7 @@ Build the workspace
.. code-block:: bash


    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

@@ -368,9 +368,9 @@ To make common command line options easier to invoke this repository makes these
 To install the default colcon mixins, run the following:

-.. code-block:: console
+.. code-block:: bash

    colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
    colcon mixin update default

 Then, try out using the ``debug`` mixin:

-.. code-block:: console
+.. code-block:: bash

    colcon build --mixin debug
