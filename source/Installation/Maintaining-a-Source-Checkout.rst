.. _MaintainingSource:

Mantener los fuentes obtenidos
==============================

.. ifconfig:: smv_current_version != '' and smv_current_version != 'rolling'

  .. note::

     Para obtener instrucciones sobre cómo mantener el código fuente obtenido de la **última versión de desarrollo** de ROS 2, consulta
     `Manteniendo el código fuente obtenido de ROS 2 Rolling <../../rolling/Installation/Maintaining-a-Source-Checkout.html>`__

.. contents::
   :depth: 2
   :local:

Si instalaste ROS 2 de fuentes, es posible que se hayan realizado cambios en el código fuente desde el momento en que lo obtuviste.
Para mantener actualizada los fuentes obtenidos, deberá actualizar periódicamente su archivo ``ros2.repos``, descargar las fuentes más recientes y recompilar su espacio de trabajo.

Actualiza tu lista de repositorios
----------------------------------

Cada versión de ROS 2 incluye un archivo ``ros2.repos`` que contiene la lista de repositorios y su versión para esa versión.



Ramas más recientes de ROS 2 {DISTRO_TITLE}
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Si deseas obtener el código más reciente para ROS 2 {DISTRO_TITLE}, puedes obtener la lista de repositorios correspondiente ejecutando:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

       cd ~/ros2_{DISTRO}
       mv -i ros2.repos ros2.repos.old
       wget https://raw.githubusercontent.com/ros2/ros2/{REPOS_FILE_BRANCH}/ros2.repos

  .. group-tab:: macOS

    .. code-block:: bash

       cd ~/ros2_{DISTRO}
       mv -i ros2.repos ros2.repos.old
       wget https://raw.githubusercontent.com/ros2/ros2/{REPOS_FILE_BRANCH}/ros2.repos

  .. group-tab:: Windows

    .. code-block:: bash

       # CMD
       cd \dev\ros2_{DISTRO}
       curl -sk https://raw.githubusercontent.com/ros2/ros2/{REPOS_FILE_BRANCH}/ros2.repos -o ros2.repos

       # PowerShell
       cd \dev\ros2_{DISTRO}
       curl https://raw.githubusercontent.com/ros2/ros2/{REPOS_FILE_BRANCH}/ros2.repos -o ros2.repos


Actualiza tus repositorios
--------------------------

Notará que en el archivo `ros2.repos <https://raw.githubusercontent.com/ros2/ros2/{REPOS_FILE_BRANCH}/ros2.repos>`__, cada repositorio tiene una ``versión`` asociada que apunta a un hash de commit, etiqueta o nombre de rama en particular.
Es posible que estas versiones se refieran a nuevas etiquetas/ramas que tu copia local de los repositorios no reconocerá porque están desactualizadas.
Debido a esto, debes actualizar los repositorios que ya has obtenido con el siguiente comando:

.. code-block:: bash

   vcs custom --args remote update

Descarga el nuevo código fuente
-------------------------------

Ahora deberías poder descargar las fuentes asociadas con la nueva lista de repositorios con:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

       vcs import src < ros2.repos
       vcs pull src

  .. group-tab:: macOS

    .. code-block:: bash

       vcs import src < ros2.repos
       vcs pull src

  .. group-tab:: Windows

    .. code-block:: bash

       # CMD
       vcs import src < ros2.repos
       vcs pull src

       # PowerShell
       vcs import --input ros2.repos src
       vcs pull src

Reocmpila tu worskpace
----------------------

Ahora que el workspace está actualizado con las fuentes más recientes, elimina tu instalación anterior y recompila tu espacio de trabajo con, por ejemplo:

.. code-block:: bash

   colcon build --symlink-install

Inspecciona tus fuentes obtenidas
---------------------------------

Durante tu desarrollo, es posible que se haya desviado del estado original de tu workspacec cuando importaste la lista de repositorios.
Si deseas conocer las versiones del conjunto de repositorios en tu workspace, puedes exportar la información mediante el siguiente comando:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

       cd ~/ros2_{DISTRO}
       vcs export src > my_ros2.repos

  .. group-tab:: macOS

    .. code-block:: bash

       cd ~/ros2_{DISTRO}
       vcs export src > my_ros2.repos

  .. group-tab:: Windows

    .. code-block:: bash

       cd \dev\ros2_{DISTRO}
       vcs export src > my_ros2.repos

Este archivo ``my_ros2.repos`` se puede compartir con otros para que puedan reproducir el estado de los repositorios en tu worskpace.
