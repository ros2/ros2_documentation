RHEL (RPM)
==========

.. contents:: Tabla de contenidos
   :depth: 2
   :local:

Los paquetes RPM para ROS 2 {DISTRO_TITLE_FULL} están actualmente disponibles para RHEL 8.
La distribución de Rolling Ridley cambiará las plataformas de destino de vez en cuando a medida que se seleccionen nuevas plataformas para el desarrollo.
Las plataformas de destino se definen en `REP 2000 <https://github.com/ros-infrastructure/rep/blob/master/rep-2000.rst>`__
La mayoría de la gente querrá usar una distribución ROS estable.

Recursos
--------

* Página de estado:

  * ROS 2 {DISTRO_TITLE} (RHEL 8): `amd64 <http://repo.ros2.org/status_page/ros_{DISTRO}_rhel.html>`__
* `Jenkins Instance <http://build.ros2.org/>`__
* `Repositories <http://repo.ros2.org>`__


Establecer configuración regional
---------------------------------

.. include:: _RHEL-Set-Locale.rst

.. _rhel-install-rpms-setup-sources:

Configurar las fuentes
----------------------

Deberás habilitar los repositorios de EPEL y el repositorio de PowerTools:

.. code-block:: bash

   sudo dnf install 'dnf-command(config-manager)' epel-release -y
   sudo dnf config-manager --set-enabled powertools

.. note:: Este paso puede ser ligeramente diferente según la distribución que esté utilizando. Consulta la documentación de EPEL: https://docs.fedoraproject.org/en-US/epel/#_quickstart

A continuación, descarga el archivo ROS 2 .repo:

.. code-block:: bash

   sudo dnf install curl
   sudo curl --output /etc/yum.repos.d/ros2.repo http://packages.ros.org/ros2/rhel/ros2.repo

Luego, actualice su caché de metadatos.
DNF puede solicitarte que verifiques la clave GPG, que debe coincidir con la ubicación ``https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc``.

.. code-block:: bash

   sudo dnf makecache

.. _rhel-install-rpms-install-ros-2-packages:

Instalar paquetes ROS 2
-----------------------

.. include:: _Dnf-Update-Admonition.rst

Instalación de escritorio (recomendada): ROS, RViz, demostraciones, tutoriales.

.. code-block:: bash

   sudo dnf install ros-{DISTRO}-desktop

ROS-Base Install (Bare Bones): bibliotecas de comunicación, paquetes de mensajes, herramientas de línea de comandos.
Sin herramientas GUI.

.. code-block:: bash

   sudo dnf install ros-{DISTRO}-ros-base

Configuración del entorno
-------------------------

Ejecutar ``source`` con el script de configuración
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Configura tu entorno ejecutando ``source`` con el siguiente archivo.

.. code-block:: bash

   # Reemplaza ".bash" en tu shell si no está usando bash
   # Los valores posibles son: setup.bash, setup.sh, setup.zsh
   source /opt/ros/{DISTRO}/setup.bash

Prueba algunos ejemplos
-----------------------

Si instalaste ``ros-{DISTRO}-desktop`` anteriormente, puede probar algunos ejemplos.

En una terminal, ejecuta ``source`` con el archivo de setup y luego ejecuta un ``talker`` de C++\:

.. code-block:: bash

   source /opt/ros/{DISTRO}/setup.bash
   ros2 run demo_nodes_cpp talker

En otra fuente de terminal, ejecuta ``source`` con el archivo de setup y luego ejecuta un ``listener`` en Python\:

.. code-block:: bash

   source /opt/ros/{DISTRO}/setup.bash
   ros2 run demo_nodes_py listener

Deberías ver al ``talker`` diciendo que está publicando (``Publishing``) mensajes y al ``listener`` diciendo que oye (``I heard``) esos mensajes.
Esto verifica que las API de C++ y Python funcionan correctamente.
¡Hurra!

Siguientes pasos después de la instalación
------------------------------------------
Continúa con los :doc:`tutoriales y demostraciones <../../Tutorials>` para configurar su entorno, crear tu propio espacio de trabajo y paquetes, y aprender los conceptos básicos de ROS 2.

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

  sudo dnf remove ros-{DISTRO}-*
