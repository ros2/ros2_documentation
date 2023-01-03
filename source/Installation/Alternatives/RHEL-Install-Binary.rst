RHEL (binarios)
===============

.. contents:: Tabla de contenidos
   :depth: 2
   :local:

Esta página explica cómo instalar ROS 2 en RHEL a partir de un paquete binario precompilado.

.. note::

    El binario precompilado no incluye todos los paquetes ROS 2.
    Se incluyen todos los paquetes de la `variante base de ROS <https://ros.org/reps/rep-2001.html#ros-base>`_ y solo un subconjunto de paquetes de la `variante de escritorio de ROS <https:/ /ros.org/reps/rep-2001.html#desktop-variants>`_ están incluidos.
    La lista exacta de paquetes se describe en los repositorios enumerados en `este archivo ros2.repos <https://github.com/ros2/ros2/blob/{REPOS_FILE_BRANCH}/ros2.repos>`_.

También hay :doc:`paquetes RPM <../RHEL-Install-RPMs>` disponibles.

Requisitos del sistema
----------------------

Actualmente soportamos RHEL 8 de 64 bits.
La distribución de Rolling Ridley cambiará las plataformas de destino de vez en cuando a medida que se seleccionen nuevas plataformas para el desarrollo.
La mayoría de la gente querrá usar una distribución ROS estable.

Habilitar repositorios requeridos
---------------------------------

La base de datos rosdep contiene paquetes de los repositorios de EPEL y PowerTools, que no están habilitados de forma predeterminada.
Se pueden habilitar ejecutando:

.. code-block:: bash

   sudo dnf install 'dnf-command(config-manager)' epel-release -y
   sudo dnf config-manager --set-enabled powertools

.. note:: Este paso puede ser ligeramente diferente según la distribución que esté utilizando. Consulte la documentación de EPEL: https://docs.fedoraproject.org/en-US/epel/#_quickstart

Instalando prerequisitos
------------------------

Hay algunos paquetes que deben instalarse para obtener y desempaquetar la versión binaria.

.. code-block:: bash

   sudo dnf install tar bzip2 wget -y

Descargando ROS 2
-----------------

No se proporcionan versiones binarias de Rolling Ridley.
En su lugar, puede descargar :ref:`prerelease binarios <Prerelease_binaries>` generados todos los días .

* Descarga el paquete más reciente para RHEL; supongamos que termina en ``~/Downloads/ros2-package-linux-x86_64.tar.bz2``.

  * Nota: puede haber más de una opción de descarga de binarios que podría causar que el nombre del archivo sea diferente.

* Decomprímelo:

  .. code-block:: bash

       mkdir -p ~/ros2_{DISTRO}
       cd ~/ros2_{DISTRO}
       tar xf ~/Downloads/ros2-package-linux-x86_64.tar.bz2

Instalación e inicialización de rosdep
--------------------------------------

.. code-block:: bash

       sudo dnf install -y python3-rosdep
       sudo rosdep init
       rosdep update

.. _rhel-install-binary-install-missing-dependencies:

Instalando las dependencias que faltan
--------------------------------------

.. include:: ../_Dnf-Update-Admonition.rst

Configur rosdistro de acuerdo con la versión que descargaste.

.. code-block:: bash

       rosdep install --from-paths ~/ros2_{DISTRO}/ros2-linux/share --ignore-src -y --skip-keys "asio cyclonedds fastcdr fastrtps ignition-cmake2 ignition-math6 python3-babeltrace python3-mypy rti-connext-dds-6.0.1 urdfdom_headers"

Instalar implementaciones de DDS adicionales (opcional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Si deseas utilizar otro proveedor de DDS o RTPS además del predeterminado, puedes encontrar instrucciones :doc:`aquí <../DDS-Implementations>`.

Configuración del entorno
-------------------------

Ejecutar ``source`` con el script de configuración
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Configura tu entorno ejecutando ``source`` con el siguiente archivo.

.. code-block:: bash

   # Reemplaza ".bash" en tu shell si no está usando bash
   # Los valores posibles son: setup.bash, setup.sh, setup.zsh
  . ~/ros2_{DISTRO}/ros2-linux/setup.bash

Prueba algunos ejemplos
-----------------------

En una terminal, ejecuta el archivo de setup y luego ejecuta un ``talker`` de C++:

.. code-block:: bash

   . ~/ros2_{DISTRO}/ros2-linux/setup.bash
   ros2 run demo_nodes_cpp talker

En otra fuente de terminal, ejecuta el archivo de setup y luego ejecuta un ``listener`` en Python:

.. code-block:: bash

   . ~/ros2_{DISTRO}/ros2-linux/setup.bash
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

Solución de problemas
---------------------

Las técnicas de resolución de problemas se pueden encontrar :doc:`aquí <../../How-To-Guides/Installation-Troubleshooting>`.

Desinstalar
-----------

1. Si instalaste tu espacio de trabajo con colcon como se indicó anteriormente, la "desinstalación" podría ser simplemente una cuestión de abrir una nueva terminal y no ejecutar el archivo ``setup`` del espacio de trabajo.
    De esta manera, su entorno se comportará como si no hubiera una instalación de {DISTRO_TITLE} en su sistema.

2. Si también está intentando liberar espacio, puede eliminar todo el directorio del espacio de trabajo con:

   .. code-block:: bash

    rm -rf ~/ros2_{DISTRO}
