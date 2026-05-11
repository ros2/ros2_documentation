.. redirect-from::

    Tutorials/Building-ROS2-Package-with-eclipse-2021-06

Building a package with VS Code (Windows 11) or Eclipse
=======================================================

.. contents:: Table of Contents
   :depth: 2
   :local:

.. warning::

   **Important Note for Windows 11 Users:**
   The Eclipse CDT indexer is currently incompatible with the isolated Pixi/MSVC environment used in Lyrical Luth. Windows users should follow the **VS Code Setup** below instead of the Eclipse instructions.

VS Code Setup (Recommended for Windows 11)
------------------------------------------

To achieve a functional development environment on Windows 11 with Pixi, follow these steps:

1. **Launch VS Code from the Pixi Shell**
   To ensure the IDE inherits the necessary environment variables (MSVC compiler and ROS 2 paths), launch it from an active terminal:
   
   .. code-block:: bash

      cd C:\dev\my_project_ws
      pixi shell --manifest-path C:\dev\lyrical\pixi.toml
      code .

2. **Install Extensions**
   Install the official **C/C++ Extension Pack** by Microsoft for IntelliSense and code navigation.

3. **Configure the Compiler Path**
   * Open the Command Palette (``Ctrl+Shift+P``) and select **C/C++: Edit Configurations (UI)**.
   * Set the **Compiler path** to your global MSVC executable:
     ``C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC/[version]/bin/Hostx64/x64/cl.exe``
   * Set the **IntelliSense mode** to ``windows-msvc-x64``.

4. **Add Recursive Include Paths**
   In the same UI menu, add these paths to the **Include path** section using recursive notation (``/**``) to resolve nested ROS 2 headers:

   * ``${workspaceFolder}/**`` [cite: 3314]
   * ``C:/dev/lyrical/rolling/install/include/**`` [cite: 3314]
   * ``C:/dev/lyrical/.pixi/envs/default/Library/include/**`` [cite: 3314]

Eclipse Setup (Linux Only)
--------------------------

*... [Original Eclipse instructions for Linux follow here] ...*
