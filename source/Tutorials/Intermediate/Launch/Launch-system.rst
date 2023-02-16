.. redirect-from::

  Tutorials/Launch-system
  Tutorials/Launch-Files/Launch-system
  Tutorials/Launch/Launch-system

Integrar ficheros de launch a paquetes de ROS 2
===============================================

**Objetivo:** Añadir un fichero de launch a un paquete de ROS 2

**Nivel del tutorial:** Intermedio

**Tiempo:** 10 minutos

.. contents:: Contenidos
   :depth: 2
   :local:

Prerequisitos
-------------

Deberías de haber repasado el tutorial sobre como :doc:`Crear un paquete de ROS 2<../../Beginner-Client-Libraries/Creating-Your-First-ROS2-Package>`.

Como siempre, no te olvides de sincronizar las fuentes de ROS 2 en :doc:`cada terminal que abra <../../Beginner-CLI-Tools/Configuring-ROS2-Environment>`.

Antecedentes
------------

En el :doc:`tutorial anterior <Creating-Launch-Files>`, vimos como escribir un fichero de launch independiente.
Este tutorial mostrará como añadir un fichero de launch a un paquete existente, y las convenciones que se suelen utilizar.

Tareas
------

1 Crear un paquete
^^^^^^^^^^^^^^^^^^

Crea un workspace donde vivirá el paquete:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

      mkdir -p launch_ws/src
      cd launch_ws/src

  .. group-tab:: macOS

    .. code-block:: bash

      mkdir -p launch_ws/src
      cd launch_ws/src

  .. group-tab:: Windows

    .. code-block:: bash

      md launch_ws\src
      cd launch_ws\src

.. tabs::

  .. group-tab:: Python package

    .. code-block:: console

      ros2 pkg create py_launch_example --build-type ament_python

  .. group-tab:: C++ package

    .. code-block:: console

      ros2 pkg create cpp_launch_example --build-type ament_cmake

2 Crear la estructura para mantener ficheros de launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Por convención, todos los ficheros de launch para un paquete son almacenados en la carpeta ``launch`` dentro del paquete.
Asegúrate de crear la carpeta ``launch`` en el nivel mas alto del paquete que creaste arriba.

.. tabs::

  .. group-tab:: Python package

    Para los paquetes de Python, la carpeta que contiene tu paquete debe verse como esto:

    .. code-block:: console

      src/
        py_launch_example/
          launch/
          package.xml
          py_launch_example/
          resource/
          setup.cfg
          setup.py
          test/

    Para que colcon pueda encontrar los ficheros de launch, necesitamos informar a las herramientas de setup de Pyton acerca de nuestros ficheros de launch usando el parámetro ``data_files`` de ``setup``.

    Dentro de nuestro fichero ``setup.py``:

    .. code-block:: python

      import os
      from glob import glob
      from setuptools import find_packages, setup

      package_name = 'py_launch_example'

      setup(
          # Other parameters ...
          data_files=[
              # ... Otros ficheros de datos
              # Incluye todos los ficheros de launch.
              (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
          ]
      )

  .. group-tab:: C++ package

    Para los paquetes de C++, solo ajustaremos el fichero de ``CMakeLists.txt`` añadiendo:

    .. code-block:: cmake

      # Install launch files.
      install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
      )

    al final del fichero (pero antes de ``ament_package()``).


3 Escribir el fichero de launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. tabs::

  .. group-tab:: Python launch file

    Dentro de tu carpeta ``launch``, crea un nuevo fichero de launch llamado ``my_script_launch.py``.
    Se recomienda``_launch.py``, aunque no es obligatorio, como sufijo para ficheros de launch de Python.
    Sin embargo, el nombre del fichero de launch necesita terminar con ``launch.py`` para ser reconocido y autocompletado por ``ros2 launch``.

    Tu fichero de launch debe definir la función ``generate_launch_description()``, la cual regresa un ``launch.LaunchDescription()`` que es usado por el verbo ``ros2 launch``.

    .. code-block:: python

      import launch
      import launch_ros.actions

      def generate_launch_description():
          return launch.LaunchDescription([
              launch_ros.actions.Node(
                  package='demo_nodes_cpp',
                  executable='talker',
                  name='talker'),
        ])

  .. group-tab:: XML launch file

    Dentro de tu carpeta ``launch``, crea un nuevo fichero de launch llamado ``my_script_launch.xml``.
    Se recomienda``_launch.xml``, aunque no es obligatorio, como sufijo para ficheros de launch de XML.

    .. code-block:: xml

      <launch>
        <node pkg="demo_nodes_cpp" exec="talker" name="talker"/>
      </launch>

  .. group-tab:: YAML launch file

    Dentro de tu carpeta ``launch``, crea un nuevo fichero de launch llamado ``my_script_launch.yaml``.
    Se recomienda``_launch.xml``, aunque no es obligatorio, como sufijo para ficheros de launch de YAML.

    .. code-block:: yaml

      launch:

      - node:
          pkg: "demo_nodes_cpp"
          exec: "talker"
          name: "talker"


4 Compilar y ejecutar el fichero de launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Ve al nivel mas alto del workspace, y compílalo:

.. code-block:: console

  colcon build

Después de que ``colcon build`` haya sido exitoso y que hayas ejecutado source en el workspace, deberías de ser capas de ejecutar el fichero de launch como sigue:

.. tabs::

  .. group-tab:: Python package

    .. tabs::

      .. group-tab:: Python launch file

        .. code-block:: console

          ros2 launch py_launch_example my_script_launch.py

      .. group-tab:: XML launch file

        .. code-block:: console

          ros2 launch py_launch_example my_script_launch.xml

      .. group-tab:: YAML launch file

        .. code-block:: console

          ros2 launch py_launch_example my_script_launch.yaml

  .. group-tab:: C++ package

    .. tabs::

      .. group-tab:: Python launch file

        .. code-block:: console

          ros2 launch cpp_launch_example my_script_launch.py

      .. group-tab:: XML launch file

        .. code-block:: console

          ros2 launch cpp_launch_example my_script_launch.xml

      .. group-tab:: YAML launch file

        .. code-block:: console

          ros2 launch cpp_launch_example my_script_launch.yaml


Documentación
-------------

`The launch documentation <https://github.com/ros2/launch/blob/{REPOS_FILE_BRANCH}/launch/doc/source/architecture.rst>`__ provides more details on concepts that are also used in ``launch_ros``.
`La documentación de launch <https://github.com/ros2/launch/blob/{REPOS_FILE_BRANCH}/launch/doc/source/architecture.rst>`__ provee mas destalles sobre los conceptos que también son usados en ``launch_ros``.

Documentación y ejemplos adicionales sobre las capacidades de launch están por venir.
De momento ve el código fuente (https://github.com/ros2/launch y https://github.com/ros2/launch_ros).
