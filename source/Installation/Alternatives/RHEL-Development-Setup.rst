.. redirect-from::

  Installation/RHEL-Development-Setup

.. _rhel-latest:

RHEL (fuentes)
==============

.. contents:: Tabla de contenidos
   :depth: 2
   :local:


Requisitos del sistema
----------------------

Las plataformas objetivo actuales de Red Hat para {DISTRO_TITLE_FULL} son:

- Tier 2: RHEL 8 64-bit

Como se define en `REP 2000 <https://www.ros.org/reps/rep-2000.html>`_

Configuración del sistema
-------------------------

Establecer configuración regional
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. include:: ../_RHEL-Set-Locale.rst

Habilitar los repositorios requeridos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

La base de datos rosdep contiene paquetes de los repositorios de EPEL y PowerTools, que no están habilitados de forma predeterminada.
Se pueden habilitar ejecutando:

.. code-block:: bash

   sudo dnf install 'dnf-command(config-manager)' epel-release -y
   sudo dnf config-manager --set-enabled powertools

.. note:: Este paso puede ser ligeramente diferente según la distribución que esté utilizando. Consulta la documentación de EPEL: https://docs.fedoraproject.org/en-US/epel/#_quickstart


Instalar herramientas de desarrollo y herramientas ROS
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   sudo dnf install -y \
     cmake \
     gcc-c++ \
     git \
     make \
     patch \
     python3-colcon-common-extensions \
     python3-pip \
     python3-pydocstyle \
     python3-pytest \
     python3-pytest-repeat \
     python3-pytest-rerunfailures \
     python3-rosdep \
     python3-setuptools \
     python3-vcstool

   # instalar algunos paquetes pip necesarios para testing y
   # no disponible como RPM
   python3 -m pip install -U --user \
     flake8-blind-except==0.1.1 \
     flake8-builtins \
     flake8-class-newline \
     flake8-comprehensions \
     flake8-deprecated \
     flake8-docstrings \
     flake8-import-order \
     flake8-quotes \
     mypy==0.931

.. _Rolling_rhel-dev-get-ros2-code:

Obtener el código ROS 2
-----------------------

Crea un espacio de trabajo y clona todos los repositorios:

.. code-block:: bash

   mkdir -p ~/ros2_{DISTRO}/src
   cd ~/ros2_{DISTRO}
   vcs import --input https://raw.githubusercontent.com/ros2/ros2/{REPOS_FILE_BRANCH}/ros2.repos src

.. _rhel-development-setup-install-dependencies-using-rosdep:

Instalar dependencias usando rosdep
-----------------------------------

.. include:: ../_Dnf-Update-Admonition.rst

.. code-block:: bash

   sudo rosdep init
   rosdep update
   rosdep install --from-paths src --ignore-src -y --skip-keys "asio cyclonedds fastcdr fastrtps ignition-cmake2 ignition-math6 python3-babeltrace python3-mypy rti-connext-dds-6.0.1 urdfdom_headers"

Instalar implementaciones de DDS adicionales (opcional)
-------------------------------------------------------

Si desea utilizar otro proveedor de DDS o RTPS además del predeterminado, puedes encontrar instrucciones :doc:`aquí <../DDS-Implementations>`.

Compilar el código en el espacio de trabajo
-------------------------------------------

Si ya instalaste ROS 2 de otra manera (ya sea a través de RPM o la distribución binaria), asegúrate de ejecutar los siguientes comandos en un entorno nuevo que no tenga otras instalaciones ejecutadas con ``source``.
También asegúrate de no tener ``source /opt/ros/${ROS_DISTRO}/setup.bash`` en su ``.bashrc``.
Puede asegurarte de que no se ha ejecutado ``source`` con ROS 2 con el comando ``printenv | grep -i ROS``.
La salida debe estar vacía.

Puedes encontrar más información sobre cómo trabajar con un espacio de trabajo de ROS en :doc:`este tutorial <../../Tutorials/Beginner-Client-Libraries/Colcon-Tutorial>`.
.. code-block:: bash

   cd ~/ros2_{DISTRO}/
   colcon build --symlink-install --cmake-args -DTHIRDPARTY_Asio=ON --no-warn-unused-cli

Nota: si estás teniendo problemas para compilar todos los ejemplos y esto te impide completar una compilación exitosa, puede usar ``COLCON_IGNORE`` de la misma manera que `CATKIN_IGNORE <https://github.com/ros-infrastructure/rep/blob/master/rep-0128.rst>`__ para ignorar el subárbol o eliminar la carpeta del espacio de trabajo.
Por ejemplo: para evitar instalar la gran biblioteca OpenCV.
Entonces simplemente ejecuta ``touch COLCON_IGNORE`` en el directorio de demo ``cam2image`` para dejarlo fuera del proceso de compilación.

Configuración del entorno
-------------------------

Obtener el script de configuración
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Configura tu entorno ejecutado ``source`` con el siguiente archivo.

.. code-block:: bash

   # Reemplaza ".bash" con tu shell si no estás usando bash
   # Los valores posibles son: setup.bash, setup.sh, setup.zsh
   . ~/ros2_{DISTRO}/install/local_setup.bash

.. _rhel_talker-listener:

Prueba algunos ejemplos
-----------------------

En una terminal, ejecuta ``source`` con el fichero de setup y luego ejecuta un ``talker`` de C++\:

.. code-block:: bash

   . ~/ros2_{DISTRO}/install/local_setup.bash
   ros2 run demo_nodes_cpp talker

En otra terminal ejecuta ``source`` con el fichero de setup y luego ejecuta un ``listener`` en Python\:

.. code-block:: bash

   . ~/ros2_{DISTRO}/install/local_setup.bash
   ros2 run demo_nodes_py listener

Deberías ver al ``talker`` diciendo que está publicando (``Publishing``) mensajes y al ``listener`` diciendo que oye (``I heard``) esos mensajes.
Esto verifica que las API de C++ y Python funcionan correctamente.
¡Hurra!

Siguientes pasos después de la instalación
------------------------------------------
Continúa con los :doc:`tutoriales y demostraciones <../../Tutorials>` para configurar tu entorno, crear tu propio espacio de trabajo y paquetes, y aprender los conceptos básicos de ROS 2.

Implementaciones adicionales de RMW (opcional)
----------------------------------------------
El middleware predeterminado que usa ROS 2 es ``Fast DDS``, pero el middleware (RMW) se puede reemplazar en tiempo de ejecución.
Consulte la :doc:`guía <../../How-To-Guides/Working-with-multiple-RMW-implementations>` sobre cómo trabajar con múltiples RMW.

Compiladores alternativos
-------------------------

Usar un compilador diferente además de gcc para compilar ROS 2 es fácil. Si establece las variables de entorno ``CC`` y ``CXX`` a ejecutables de un compilador de C y C++ en funcionamiento, respectivamente, y vuelve a activar la configuración de CMake (usando ``--force-cmake-config`` o eliminando los paquetes que deseas que se vean afectados), CMake reconfigurará y usará el compilador diferente.

Clang
^^^^^

Para configurar CMake para detectar y usar Clang:

.. code-block:: bash

   sudo dnf install clang
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
