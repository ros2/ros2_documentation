.. redirect-from::

  Installation/Rolling/OSX-Development-Setup
  Installation/macOS-Development-Setup

.. _macOS-latest:

macOS (fuentes)
===============

.. contents:: Tabla de contenidos
   :depth: 2
   :local:

Requisitos del sistema
----------------------

Actualmente damos soporte a macOS Mojave (10.14).
La distribución de Rolling Ridley cambiará las plataformas de destino de vez en cuando a medida que haya nuevas plataformas disponibles.
La mayoría de la gente querrá usar una distribución ROS estable.

Instalar requisitos previos
---------------------------

Necesitas las siguientes cosas instaladas para compilar ROS 2:


#.
   **Xcode**

   * Si aún no lo tienes instalado, instala Xcode.
   * Nota: Las versiones de Xcode posteriores a la 11.3.1 ya no se pueden instalar en macOS Mojave, por lo que debes instalar una versión anterior manualmente, consulta: https://stackoverflow.com/a/61046761
   * Además, si aún no lo tienes instalado, instala las herramientas de línea de comandos:

     .. code-block:: bash

        xcode-select --install
        # This command will not succeed if you have not installed Xcode.app
        sudo xcode-select --switch /Applications/Xcode.app/Contents/Developer
        # If you installed Xcode.app manually, you need to either open it or run:
        sudo xcodebuild -license
        # To accept the Xcode.app license

#.
   **brew** *(se necesita para instalar más cosas; probablemente ya tengas esto)*:


   * Sigue las instrucciones de instalación en http://brew.sh/
   *
     *Optional*: Check that ``brew`` is happy with your system configuration by running:

     .. code-block:: bash

        brew doctor

     Soluciona cualquier problema que identifique.

#.
   Usa ``brew`` para instalar más cosas:

   .. code-block:: bash

       brew install asio assimp bison bullet cmake console_bridge cppcheck \
         cunit eigen freetype graphviz opencv openssl orocos-kdl pcre poco \
         pyqt5 python qt@5 sip spdlog tinyxml tinyxml2

#.
   Configura algunas variables de entorno:

   .. code-block:: bash

       # Agrega el directorio openssl para DDS-Security
       # si estás usando ZSH, reemplace '.bashrc' por '.zshrc'
       echo "export OPENSSL_ROOT_DIR=$(brew --prefix openssl)" >> ~/.bashrc

       # Agrega el directorio Qt a PATH y CMAKE_PREFIX_PATH
       export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/usr/local/opt/qt@5
       export PATH=$PATH:/usr/local/opt/qt@5/bin

#.
   Usa ``python3 -m pip`` (solo ``pip`` puede instalar Python3 o Python2) para instalar más cosas:

   .. code-block:: bash

       python3 -m pip install -U \
        argcomplete catkin_pkg colcon-common-extensions coverage \
        cryptography empy flake8 flake8-blind-except==0.1.1 flake8-builtins \
        flake8-class-newline flake8-comprehensions flake8-deprecated \
        flake8-docstrings flake8-import-order flake8-quotes \
        importlib-metadata jsonschema lark==1.1.1 lxml matplotlib mock mypy==0.931 netifaces \
        nose pep8 psutil pydocstyle pydot pygraphviz pyparsing==2.4.7 \
        pytest-mock rosdep rosdistro setuptools==59.6.0 vcstool

   Asegúrate que la variable de entorno ``$PATH`` contenga la ubicación de instalación de los binarios (predeterminado: ``$HOME/Library/Python/<version>/bin``)

#.
   *Opcional*: si deseas construir el bridge ROS 1<->2, también debes instalar ROS 1:


   * Comienza con las instrucciones de instalación normales:
   *
     Cuando llega al paso en el que llama a ``rosinstall_generator`` para obtener el código fuente, aquí hay una invocación alternativa que trae solo el mínimo requerido para producir un bridge útil:

     .. code-block:: bash

          rosinstall_generator catkin common_msgs roscpp rosmsg --rosdistro kinetic --deps --wet-only --tar > kinetic-ros2-bridge-deps.rosinstall
          wstool init -j8 src kinetic-ros2-bridge-deps.rosinstall


     De lo contrario, simplemente sigue las instrucciones normales, luego ejecuta ``source`` con el ``install_isolated/setup.bash`` resultante antes de proceder a compilar ROS 2.

Deshabilitar la protección de integridad del sistema (SIP)
----------------------------------------------------------

Las versiones de macOS/OS X >=10.11 tienen la Protección de integridad del sistema habilitada de manera predeterminada.
Para que SIP no impida que los procesos hereden variables de entorno del enlazador dinámico, como ``DYLD_LIBRARY_PATH``, debes desactivarlo `siguiendo estas instrucciones <https://developer.apple.com/library/content/documentation /Security/Conceptual/System_Integrity_Protection_Guide/ConfiguringSystemIntegrityProtection/ConfiguringSystemIntegrityProtection.html>`__.

Obtener el código ROS 2
-----------------------

Crea un espacio de trabajo y clona todos los repositorios:

.. code-block:: bash

   mkdir -p ~/ros2_{DISTRO}/src
   cd ~/ros2_{DISTRO}
   vcs import --input https://raw.githubusercontent.com/ros2/ros2/{REPOS_FILE_BRANCH}/ros2.repos src

Instalar proveedores de DDS adicionales (opcional)
--------------------------------------------------

Si deseas utilizar otro proveedor de DDS o RTPS además del predeterminado, puede encontrar instrucciones :doc:`aquí <../DDS-Implementations>`.

Construir el código ROS 2
-------------------------

Ejecuta la herramienta ``colcon`` para compilar todo (más información sobre el uso de ``colcon`` en :doc:`este tutorial <../../Tutorials/Beginner-Client-Libraries/Colcon-Tutorial>`):
.. code-block:: bash

   cd ~/ros2_{DISTRO}/
   colcon build --symlink-install --packages-skip-by-dep python_qt_binding

Nota: debido a un problema no resuelto con SIP, Qt@5 y PyQt5, debemos deshabilitar ``python_qt_binding`` para que la compilación tenga éxito.
Esto se eliminará cuando se resuelva el problema, consulte: https://github.com/ros-visualization/python_qt_binding/issues/103

Configuración del entorno
-------------------------

Ejecuta ``source`` con el archivo de setup de ROS 2:

.. code-block:: bash

   . ~/ros2_{DISTRO}/install/setup.bash

Esto configurará automáticamente el entorno para cualquier proveedor de DDS para el que se haya compilado soporte.

Prueba algunos ejemplos
-----------------------

En una terminal, configura el entorno ROS 2 como se describe arriba y luego ejecuta un ``talker`` de C++:

.. code-block:: bash

   ros2 run demo_nodes_cpp talker

En otra terminal, llama a ``source``con el archivo de setup y luego ejecuta un ``listener`` en Python:

.. code-block:: bash

   ros2 run demo_nodes_py listener

Deberías ver al ``talker`` diciendo que está publicando (``Publishing``) mensajes y al ``listener`` diciendo que oye (``I heard``) esos mensajes.
Esto verifica que las API de C++ y Python funcionan correctamente.
¡Hurra!

Siguientes pasos después de la instalación
------------------------------------------

Continúa con los `tutoriales y demostraciones <../../Tutorials>` para configurar su entorno, crear su propio espacio de trabajo y paquetes, y aprender los conceptos básicos de ROS 2.

Usando el bridge de ROS 1
-------------------------

El puente de ROS 1 puede conectar topics de ROS 1 a ROS 2 y viceversa. Consulta la `documentación <https://github.com/ros2/ros1_bridge/blob/master/README.md>`__ específica sobre cómo construir y usar el bridge de ROS 1.

Implementaciones adicionales de RMW (opcional)
----------------------------------------------

El middleware predeterminado que usa ROS 2 es ``Fast DDS``, pero el middleware (RMW) se puede reemplazar en tiempo de ejecución.
Consulta la :doc:`guía <../../How-To-Guides/Working-with-multiple-RMW-implementations>` sobre cómo trabajar con múltiples RMW.

Estar al día
------------

Consulta :doc:`../Maintaining-a-Source-Checkout` para actualizar periódicamente la instalación de fuentes.

Solución de problemas
---------------------

Las técnicas de resolución de problemas se pueden encontrar :ref:`aquí <macOS-troubleshooting>`.

Desinstalar
-----------

1. Si instalaste tu espacio de trabajo con colcon como se indicó anteriormente, la "desinstalación" podría ser simplemente una cuestión de abrir una nueva terminal y no ejecutar ``source```  con el archivo ``setup`` del espacio de trabajo.
    De esta manera, su entorno se comportará como si no hubiera una instalación de {DISTRO_TITLE} en su sistema.

2. Si también estás intentando liberar espacio, puede eliminar todo el directorio del espacio de trabajo con:

   .. code-block:: bash

    rm -rf ~/ros2_{DISTRO}
