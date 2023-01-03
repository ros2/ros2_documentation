Ubuntu (binarios)
=================

.. contents:: Tabla de contenidos
   :depth: 2
   :local:

Esta página explica cómo instalar ROS 2 en Ubuntu Linux desde un paquete binario precompilado.

.. note::

    El binario precompilado no incluye todos los paquetes ROS 2.
    Se incluyen todos los paquetes de la `variante base de ROS <https://ros.org/reps/rep-2001.html#ros-base>`_ y solo un subconjunto de paquetes de la `variante de escritorio ROS <https:/ /ros.org/reps/rep-2001.html#desktop-variants>`_ están incluidos.
    La lista exacta de paquetes se describe en los repositorios enumerados en `este archivo ros2.repos <https://github.com/ros2/ros2/blob/{REPOS_FILE_BRANCH}/ros2.repos>`_.

Requisitos del sistema
----------------------

Solo Windows 10 está soportado.

.. _windows-install-binary-installing-prerequisites:

.. include:: _Windows-Install-Prerequisites.rst

Descargar ROS 2
---------------

No se proporcionan versiones binarias de {DISTRO_TITLE_FULL}.
En su lugar, puede descargar versiones diarias de :ref:`prerelease binarios <Prerelease_binaries>`.

* Descarga el paquete más reciente para Windows, por ejemplo, ``ros2-package-windows-AMD64.zip``.

.. note::

    Puede haber más de una opción de descarga binaria que podría causar que el nombre del archivo sea diferente.

.. note::

    Para instalar bibliotecas de depuración para ROS 2, consulta `Cosas adicionales para depuración`_.
    Luego continúa descargando ``ros2-package-windows-debug-AMD64.zip``.

* Descomprima el archivo zip en algún lugar (supondremos ``C:\dev\ros2_{DISTRO}``\ ).

Configuración del entorno
-------------------------

Inicia una shell de comandos y obtén el archivo de configuración de ROS 2 para configurar el espacio de trabajo:

.. code-block:: bash

   call C:\dev\ros2_{DISTRO}\local_setup.bat

Es normal que el comando anterior, si nada más salió mal, muestre "El sistema no puede encontrar la ruta especificada". Exactamente una vez.

Prueba algunos ejemplos
-----------------------

En un shell de comandos, configure el entorno ROS 2 como se describe arriba y luego ejecute un ``talker`` de C++\:

.. code-block:: bash

   ros2 run demo_nodes_cpp talker

Abre otro shell de comandos y ejecuta un ``listener`` Python\ :

.. code-block:: bash

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

    rmdir /s /q \ros2_{DISTRO}

Cosas adicionales para depuración
---------------------------------

Para descargar las bibliotecas de depuración de ROS 2, deberás descargar ``ros2-{DISTRO}-*-windows-debug-AMD64.zip``.
Ten en cuenta que las bibliotecas de depuración requieren una configuración adicional adicional para funcionar como se indica a continuación.

La instalación de Python puede requerir modificaciones para habilitar los símbolos de depuración y los binarios de depuración:

* Busca en la **Barra de búsqueda** de Windows y abra **Aplicaciones y características**.
* Busca la versión de Python instalada.

* Haz clic en  Modifica.

      .. image:: images/python_installation_modify.png
         :width: 500 px

* Haz clic en Siguiente para ir a **Opciones avanzadas**.

      .. image:: images/python_installation_next.png
         :width: 500 px

* Asegúrate de que **Descargar símbolos de depuración** y **Descargar archivos binarios de depuración** estén marcados.

      .. image:: images/python_installation_enable_debug.png
         :width: 500 px

* Haz clic en Instalar.

(Alternativa) Instalación de ROS 2 compilado desde aka.ms/ros
-------------------------------------------------------------

El proyecto https://aka.ms/ros aloja compilaciones de ROS 2 en las instantáneas de release.
Esta sección explica cómo instalar ROS 2 desde este canal.

Instalar compilaciones de ROS 2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

En un símbolo del sistema administrativo, ejecute los siguientes comandos.

.. code-block:: bash

   mkdir c:\opt\chocolatey
   set PYTHONNOUSERSITE=1
   set ChocolateyInstall=c:\opt\chocolatey
   choco source add -n=ros-win -s="https://aka.ms/ros/public" --priority=1
   choco upgrade ros-foxy-desktop -y --execution-timeout=0

Configuración del entorno
^^^^^^^^^^^^^^^^^^^^^^^^^

Inicia un símbolo del sistema administrativo y ejecuta ``source`` con archivo de configuración de ROS 2 para configurar el espacio de trabajo:

.. code-block:: bash

   call C:\opt\ros\foxy\x64\local_setup.bat

Estar al día
^^^^^^^^^^^^

Para mantenerse actualizado con las últimas compilaciones, ejecute:

.. code-block:: bash

   set ChocolateyInstall=c:\opt\chocolatey
   choco upgrade all -y --execution-timeout=0

Desinstalar
^^^^^^^^^^^

Si deseas eliminar por completo el entorno descargado anteriormente, ejecuta este comando:

.. code-block:: bash

   rmdir /s /q C:\opt\
