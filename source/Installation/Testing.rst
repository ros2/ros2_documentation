.. redirect-from::

   Installation/Prerelease-Testing

Pruebas con archivos binarios pre-relase
========================================

Muchos paquetes de ROS se proporcionan como binarios precompilados.
Por lo general, obtendrás la versión publicada de los archivos binarios siguiendo :doc:`../Installation`.
También hay versiones pre-relase de binarios que son útiles para hacer pruebas antes de hacer un lanzamiento oficial.
Este artículo describe varias opciones si deseas probar versiones pre-relase de los binarios de ROS.

Repositorio de pruebas de Debian
--------------------------------

Cuando los paquetes se liberan en una distribución ROS (usando bloom), buildfarm los compila en paquetes de Debian que se almacenan temporalmente en el repositorio de **compilación** apt.
A medida que se compilan los paquetes dependientes, un proceso automático sincroniza periódicamente los paquetes en **compilación** con un repositorio secundario llamado **ros-testing**.
**ros-testing** está pensado como un área de inmersión en la que los desarrolladores y los usuarios avanzados pueden realizar pruebas adicionales a los paquetes, antes de que se sincronicen manualmente con el repositorio público de ros desde el que los usuarios suelen instalar los paquetes.

Aproximadamente cada dos semanas, el administrador de versiones de rosdistro sincroniza manualmente el contenido de **ros-testing** en el repositorio **main** de ROS.

Para los sistemas operativos basados en Debian, puedes instalar paquetes binarios desde el repositorio **ros-testing**.

1. Asegúrese de tener una instalación de ROS 2 en funcionamiento desde paquetes de Debian (consulte :doc:`../Installation`).
2. Edita (con sudo) el archivo ``/etc/apt/sources.list.d/ros2-latest.list`` y cambia ``ros2`` por ``ros2-testing``.
   Por ejemplo, en Ubuntu Jammy, el contenido debería tener el siguiente aspecto:

   .. code-block:: sh

      # deb http://packages.ros.org/ros2/ubuntu jammy main
      deb http://packages.ros.org/ros2-testing/ubuntu jammy main

3. Actualiza el índice ``apt``:

   .. code-block:: sh

      sudo apt update

4. Ahora puedes instalar paquetes individuales desde el repositorio de prueba, por ejemplo:

   .. code-block:: sh

      sudo apt install ros-{DISTRO}-my-just-released-package

5. Alternativamente, puedes mover toda su instalación de ROS 2 al repositorio de prueba:

   .. code-block:: sh

      sudo apt dist-upgrade

6. Una vez que hayas terminado de probar, puedes volver al repositorio normal cambiando el contenido de ``/etc/apt/sources.list.d/ros2-latest.list``:

   .. code-block:: sh

      deb http://packages.ros.org/ros2/ubuntu jammy main
      # deb http://packages.ros.org/ros2-testing/ubuntu jammy main

   y haciendo un update y upgrade:

   .. code-block:: sh

      sudo apt update
      sudo apt dist-upgrade

.. _Prerelease_binaries:

Binarios pesados (Fat binaries)
-------------------------------

Para los paquetes principales, ejecutamos trabajos de empaquetado nocturnos para Ubuntu Linux, RHEL y Windows.
Estos trabajos de empaquetado producen archivos con binarios precompilados que se pueden descargar y extraer a tu sistema de archivos.

1. Asegúrese de tener todas las dependencias instaladas de acuerdo con la :doc:`última configuración de desarrollo <Alternatives/Latest-Development-Setup>` para tu plataforma.

2. Ve a https://ci.ros2.org/view/packaging/ y seleccione un trabajo de empaque de la lista correspondiente a tu plataforma.

3. Debajo del encabezado "Últimos artefactos exitosos"("Last Successful Artifacts"), deberías ver un enlace de descarga (por ejemplo, para Windows, ``ros2-package-windows-AMD64.zip``).

4. Descarga y extráe el archivo a tu sistema de archivos.

5. Para usar la instalación binaria pesada, obtén el archivo ``setup.*`` que se puede encontrar en la raíz del archivo.

   .. tabs::

     .. group-tab:: Ubuntu Linux and RHEL

       .. code-block:: sh

          source path/to/extracted/archive/setup.bash

     .. group-tab:: Windows

       .. code-block:: sh

          call path\to\extracted\archive\setup.bat

Docker
------

Para Ubuntu Linux, también hay una imagen Docker nocturna basada en el archivo fat nocturno.

1. Extráe la imagen de Docker:

   .. code-block:: sh

      docker pull osrf/ros2:nightly

2. Inicia un contenedor interactivo:

   .. code-block:: sh

      docker run -it osrf/ros2:nightly

Para obtener asistencia sobre la ejecución de aplicaciones GUI en Docker, consulta el tutorial `Usar GUIs con Docker <https://wiki.ros.org/docker/Tutorials/GUI>`_ o la herramienta `rocker <https://github .com/osrf/rocker>`_.
