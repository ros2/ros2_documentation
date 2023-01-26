.. redirect-from::

    Tutorials/Launch-Files/Launch-Main
    Tutorials/Launch/Launch-Main

.. _LaunchFilesMain:

Launch
======

Los ficheros de ROS 2 launch te permiten iniciar y configurar varios ejecutables que contienen nodos de ROS 2 simultáneamente.

.. toctree::
   :hidden:

   Creating-Launch-Files
   Launch-system
   Using-Substitutions
   Using-Event-Handlers
   Using-ROS2-Launch-For-Large-Projects

#. :doc:`Crear un fichero de launch<./Creating-Launch-Files>`.

   Aprende como crear un fichero de launch que iniciará nodos y sus configuraciones de una sola vez.

#. :doc:`Launching y monitorizado de multiples nodos <./Launch-system>`.

   Obtén una visión general mas avanzada de como funcionan los ficheros de launch.

#. :doc:`Utilizar substituciones <./Using-Substitutions>`.

   Usa substituciones para proveer mayor flexibilidad cuando describas ficheros launch reusables.

#. :doc:`Utilizar controladores de evento <./Using-Event-Handlers>`.

   Usa controladores de evento para monitorizar el estado de los procesos o para definir un conjunto complejo de reglas que pueden ser utilizados para modificar de forma dinámica los ficheros de launch.

#. :doc:`Gestión de proyectos grandes <./Using-ROS2-Launch-For-Large-Projects>`.

   Estructura ficheros de launch para proyectos grandes para poder reutilizarlos tanto como sea posible en diferentes situaciones.
   Ve ejemplos de uso de diferentes herramientas de launch como parámetros, ficheros YAML, remappings, namespaces, argumentos por defecto, y configuraciones de RViz.

.. note::

   Si tu vienes de ROS 1, puedes ver el :doc:`ROS Launch guía de migración <../../../How-To-Guides/Launch-files-migration-guide>` para ayudarte a migrar tus ficheros launch a ROS 2.
