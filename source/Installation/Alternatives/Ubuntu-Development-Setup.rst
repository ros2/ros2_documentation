.. _linux-latest:

.. redirect-from::

   Installation/Linux-Development-Setup
   Installation/Ubuntu-Development-Setup

Ubuntu (fuentes)
================

.. contents:: Tabla de Contenidos
   :depth: 2
   :local:


Requisitos del sistema
----------------------
Las plataformas de destino actuales basadas en Debian para {DISTRO_TITLE_FULL} son:

- Tier 1: Ubuntu Linux - Jammy (22.04) 64-bit
- Tier 3: Ubuntu Linux - Focal (20.04) 64-bit
- Tier 3: Debian Linux - Bullseye (11) 64-bit


Otras plataformas Linux con diferentes niveles de soporte incluyen:

- Arch Linux, ver `instructions alternativas <https://wiki.archlinux.org/index.php/ROS#ROS_2>`__
- Fedora Linux, ver :doc:`instructions alternativas <Fedora-Development-Setup>`
- OpenEmbedded / webOS OSE, ver `instructions alternativas <https://github.com/ros/meta-ros/wiki/OpenEmbedded-Build-Instructions>`__

Como está definido en `REP 2000 <https://www.ros.org/reps/rep-2000.html>`_.

Configuración del sistema
-------------------------

Establecer configuración regional
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. include:: ../_Ubuntu-Set-Locale.rst

Agregar el repositorio apt de ROS 2

.. include:: ../_Apt-Repositories.rst

Instalar herramientas de desarrollo y herramientas ROS
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Instalar paquetes comunes.

.. code-block:: bash

   sudo apt update && sudo apt install -y \
     python3-flake8-docstrings \
     python3-pip \
     python3-pytest-cov \
     ros-dev-tools

Instala paquetes según tu versión de Ubuntu.

.. tabs::

   .. group-tab:: Ubuntu 22.04 LTS and later

      .. code-block:: console

         sudo apt install -y \
            python3-flake8-blind-except \
            python3-flake8-builtins \
            python3-flake8-class-newline \
            python3-flake8-comprehensions \
            python3-flake8-deprecated \
            python3-flake8-import-order \
            python3-flake8-quotes \
            python3-pytest-repeat \
            python3-pytest-rerunfailures

   .. group-tab:: Ubuntu 20.04 LTS

      .. code-block:: bash

         python3 -m pip install -U \
            flake8-blind-except \
            flake8-builtins \
            flake8-class-newline \
            flake8-comprehensions \
            flake8-deprecated \
            flake8-import-order \
            flake8-quotes \
            "pytest>=5.3" \
            pytest-repeat \
            pytest-rerunfailures


.. _Rolling_linux-dev-get-ros2-code:

Obtener el código ROS 2
-----------------------

Crea un espacio de trabajo y clona todos los repositorios:

.. code-block:: bash

   mkdir -p ~/ros2_{DISTRO}/src
   cd ~/ros2_{DISTRO}
   vcs import --input https://raw.githubusercontent.com/ros2/ros2/{REPOS_FILE_BRANCH}/ros2.repos src

.. _linux-development-setup-install-dependencies-using-rosdep:

Instalar dependencias usando rosdep
-----------------------------------

.. include:: ../_Apt-Upgrade-Admonition.rst

.. code-block:: bash

   sudo rosdep init
   rosdep update
   rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

.. include:: ../_rosdep_Linux_Mint.rst

Instalar implementaciones de DDS adicionales (opcional)
-------------------------------------------------------

Si desea utilizar otro proveedor de DDS o RTPS además del predeterminado, puedes encontrar instrucciones :doc:`aquí <../DDS-Implementations>`.

Compilar el código en el espacio de trabajo
-------------------------------------------

Si ya instaste ROS 2 de otra manera (ya sea a través de Debian o la distribución binaria), asegúrate de ejecutar los siguientes comandos en un entorno nuevo que no hayas ejecutado ``source`` en otras instalaciones.
También asegúrate de no tener ``source /opt/ros/${ROS_DISTRO}/setup.bash`` en tu ``.bashrc``.
Puede asegurarte de que no has ejecutado ``source`` con ROS 2 con el comando ``printenv | grep -i ROS``.
La salida debe estar vacía.

Puede encontrar más información sobre cómo trabajar con un espacio de trabajo de ROS en :doc:`este tutorial <../../Tutorials/Beginner-Client-Libraries/Colcon-Tutorial>`.
.. code-block:: bash

   cd ~/ros2_{DISTRO}/
   colcon build --symlink-install

Nota: si tienes problemas para compilar todos los ejemplos y esto le impide completar una compilación exitosa, puedes usar ``COLCON_IGNORE`` de la misma manera que `CATKIN_IGNORE <https://github.com/ros-infrastructure/rep/blob/master/rep-0128.rst>`__ para ignorar el subárbol o eliminar la carpeta del espacio de trabajo.
Por ejemplo: te gustaría evitar instalar la gran biblioteca OpenCV.
Entonces simplemente ejecuta ``touch COLCON_IGNORE`` en el directorio de demo ``cam2image`` para dejarlo fuera del proceso de compilación.

Configuración del entorno
-------------------------

Obtener el script de configuración
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Configura su entorno obteniendo el siguiente archivo.

.. code-block:: bash

   # Reemplaza ".bash" en tu shell si no está usando bash
   # Los valores posibles son: setup.bash, setup.sh, setup.zsh
   . ~/ros2_{DISTRO}/install/local_setup.bash

.. _talker-listener:

Prueba algunos ejemplos
-----------------------

En una terminal, ejecuta el archivo de setup y luego ejecuta un ``talker`` de C++\:

.. code-block:: bash

   . ~/ros2_{DISTRO}/install/local_setup.bash
   ros2 run demo_nodes_cpp talker

En otra termnal, ejecuta el archivo de setup y luego ejecuta un ``listener`` en Python\:

.. code-block:: bash

   . ~/ros2_{DISTRO}/install/local_setup.bash
   ros2 run demo_nodes_py listener

Deberías ver al ``talker`` diciendo que está publicando (``Publishing``) mensajes y al ``listener`` diciendo que oye (``I heard``) esos mensajes.
Esto verifica que las API de C++ y Python funcionan correctamente.
¡Hurra!

Siguientes pasos después de la instalación
------------------------------------------
Continúa con los :doc:`tutoriales y demostraciones <../../Tutorials>` para configurar su entorno, crear tu propio espacio de trabajo y paquetes, y aprender los conceptos básicos de ROS 2.

Usando el bridge ROS 1
----------------------
El bridge ROS 1 puede conectar topics de ROS 1 a ROS 2 y viceversa. Consulta la `documentación <https://github.com/ros2/ros1_bridge/blob/master/README.md>`__ específica sobre cómo construir y usar el puente ROS 1.

Implementaciones adicionales de RMW (opcional)
----------------------------------------------
El middleware predeterminado que usa ROS 2 es ``Fast DDS``, pero el middleware (RMW) se puede reemplazar en tiempo de ejecución.
Consulta la :doc:`guía <../../How-To-Guides/Working-with-multiple-RMW-implementations>` sobre cómo trabajar con múltiples RMW.


Compiladores alternativos
-------------------------

Usar un compilador diferente además de gcc para compilar ROS 2 es fácil. Si establece las variables de entorno ``CC`` y ``CXX`` a ejecutables de un compilador de C y C++ en funcionamiento, respectivamente, y dispara otra vez la configuración de CMake (usando ``--force-cmake-config`` o eliminando los paquetes que desea que se vean afectados), CMake reconfigurará y usará el compilador diferente.

Clang
^^^^^

Para configurar CMake para detectar y usar Clang:

.. code-block:: bash

   sudo apt install clang
   export CC=clang
   export CXX=clang++
   colcon build --cmake-force-configure

Estar al día
------------

Consulta :doc:`../Maintaining-a-Source-Checkout` para actualizar periódicamente la instalación de fuentes.

Solución de problemas
---------------------

Las técnicas de solución de problemas se pueden encontrar :ref:`aquí <linux-troubleshooting>`.

Desinstalar
-----------

1. Si instalaste tu espacio de trabajo con colcon como se indicó anteriormente, la "desinstalación" podría ser simplemente una cuestión de abrir una nueva terminal y no ejecutar ``source```  con el archivo ``setup`` del espacio de trabajo.
    De esta manera, su entorno se comportará como si no hubiera una instalación de {DISTRO_TITLE} en su sistema.

2. Si también estás intentando liberar espacio, puede eliminar todo el directorio del espacio de trabajo con:

   .. code-block:: bash

    rm -rf ~/ros2_{DISTRO}
