.. redirect-from::

   Installation/Linux-Install-Debians

Ubuntu (Debian)
===============

.. contents:: Table of Contents
   :depth: 2
   :local:

Los paquetes de Debian para ROS 2 {DISTRO_TITLE_FULL} están actualmente disponibles para Ubuntu Jammy.
La distribución de Rolling Ridley cambiará las plataformas de destino de vez en cuando a medida que se seleccionen nuevas plataformas para el desarrollo.
Las plataformas de destino se definen en `REP 2000 <https://github.com/ros-infrastructure/rep/blob/master/rep-2000.rst>`__
La mayoría de la gente querrá usar una distribución ROS estable.

Recursos
--------

* Página de estado:

  * ROS 2 {DISTRO_TITLE} (Ubuntu Jammy): `amd64 <http://repo.ros2.org/status_page/ros_{DISTRO}_default.html>`__\ , `arm64 <http://repo.ros2.org/status_page/ros_{DISTRO}_ujv8.html>`__
  * `Instancia de Jenkins <http://build.ros2.org/>`__
  * `Repositorios <http://repo.ros2.org>`__


Establecer configuración regional
---------------------------------

.. include:: _Ubuntu-Set-Locale.rst

.. _linux-install-debians-setup-sources:

Configurar las fuentes
----------------------

.. include:: _Apt-Repositories.rst

.. _linux-install-debians-install-ros-2-packages:

Instalar paquetes ROS 2
-----------------------

Actualiza los cachés de tus repositorios apt después de configurar los repositorios.

.. code-block:: bash

   sudo apt update

.. include:: _Apt-Upgrade-Admonition.rst

.. warning::

   Debido a las primeras actualizaciones en Ubuntu 22.04, es importante que los paquetes relacionados con ``systemd`` y ``udev`` se actualicen antes de instalar ROS 2.
   La instalación de las dependencias de ROS 2 en un sistema recién instalado sin actualizar puede desencadenar la **eliminación de paquetes críticos del sistema**.

   Por favor dirígete a `ros2/ros2#1272 <https://github.com/ros2/ros2/issues/1272>`_ y `Launchpad #1974196 <https://bugs.launchpad.net/ubuntu/+source/systemd/+bug/1974196>`_ para más información.

Instalación de escritorio (recomendada): ROS, RViz, demostraciones, tutoriales.

.. code-block:: bash

   sudo apt install ros-{DISTRO}-desktop

ROS-Base Install (Bare Bones): bibliotecas de comunicación, paquetes de mensajes, herramientas de línea de comandos.
Sin herramientas GUI.

.. code-block:: bash

   sudo apt install ros-{DISTRO}-ros-base

Herramientas de desarrollo: Compiladores y otras herramientas para construir paquetes ROS.

.. code-block:: bash

   sudo apt install ros-dev-tools

Configuración del entorno
-------------------------

Obtener el script de configuración
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Configura tu entorno llamando a ``source`` con el siguiente archivo.

.. code-block:: bash

   # Reemplaza ".bash" en tu shell si no está usando bash
   # Los valores posibles son: setup.bash, setup.sh, setup.zsh
   source /opt/ros/{DISTRO}/setup.bash

Prueba algunos ejemplos
-----------------------

Talker-listener
^^^^^^^^^^^^^^^

Si instalaste anteriormente ``ros-{DISTRO}-desktop``, puedes probar algunos ejemplos.

En una terminal, llama a ``source`` con el archivo de setup y luego ejecuta un ``talker`` de C++\:

.. code-block:: bash

   source /opt/ros/{DISTRO}/setup.bash
   ros2 run demo_nodes_cpp talker

En otra fuente de terminal, llama a ``source`` con el archivo de setup y luego ejecuta un ``listener`` en Python\:

.. code-block:: bash

   source /opt/ros/{DISTRO}/setup.bash
   ros2 run demo_nodes_py listener

Deberías ver al ``talker`` diciendo que está ``Publishing`` mensajes y al ``listener`` diciendo ``I heard`` esos mensajes.
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
Consulta la :doc:`guía <../How-To-Guides/Working-with-multiple-RMW-implementations>` sobre cómo trabajar con múltiples RMW.

Solución de problemas
---------------------

Las técnicas de resolución de problemas se pueden encontrar :doc:`aquí <../How-To-Guides/Installation-Troubleshooting>`.

Desinstalar
-----------

Si necesitas desinstalar ROS 2 o cambiar a una instalación basada en fuentes una vez que
ya has instalado desde binarios, ejecuta el siguiente comando:

.. code-block:: bash

  sudo apt remove ~nros-{DISTRO}-* && sudo apt autoremove

También es posible que desees eliminar el repositorio:

.. code-block:: bash

  sudo rm /etc/apt/sources.list.d/ros2.list
  sudo apt update
  sudo apt autoremove
  # Considere la posibilidad de actualizar los paquetes previamente ocultados.
  sudo apt upgrade
