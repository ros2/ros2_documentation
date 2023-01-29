.. redirect-from::

  Contributing

.. _Contributing:

contribuyendo
=============

.. contents:: Tabla de Contenido
   :depth: 1
   :local:

Algunas cosas para recordar antes de comenzar a contribuir al proyecto de ROS 2.

Principios
----------

* Respeta lo que vino antes

  ROS existe desde hace más de una década y es utilizado por desarrolladores de todo el mundo.
  Mantén una actitud humilde y una mentalidad abierta mientras contribuyes.

* Involucrar a Open Robotics lo antes posible

   * Open Robotics actúa como guardián y defensor de la comunidad de ROS.
     Apóyate en su experiencia y criterio técnico desde la fase de diseño.
   * Inicia conversaciones con Open Robotics y la comunidad desde el principio.
     Los contribuidores de ROS pueden tener una visión más clara del panorama general.
     Si implementas una función y envías un pull request sin discutirlo primero con la comunidad, corres el riesgo de que sea rechazado o puede que se te pida que reconsideres gran parte de tu diseño.
   * Por lo general, es preferible abrir issues o usar Discourse para socializar una idea antes de comenzar la implementación.

* Adoptar las mejores prácticas de la comunidad siempre que sea posible en lugar de procesos extrabagantes (ad-hoc)

   Piensa en la experiencia del usuario final al momento desarrollar y contribuir.
   Evita el uso de herramientas o bibliotecas no estándar que pueden no ser accesibles para todos.

* Piensa en la comunidad como un todo

   Piense en el panorama general.
   Hay desarrolladores que construyen diferentes robots con diferentes restricciones.
   ROS necesita adaptarse a los requisitos de toda la comunidad.

Hay varias formas de contribuir al proyecto ROS 2.

Discusiones y soporte
---------------------

Algunas de las formas más fáciles de contribuir a ROS 2 implican participar en debates y apoyo de la comunidad.
Puede encontrar más información sobre cómo comenzar en la página :doc:`Contacto <../../Contact>`.

Contribuir al Código 
--------------------

Configuración de tu entorno de desarrollo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Para comenzar, debes instalar a partir del código  fuente; siga :ref:`las instrucciones de instalación a partir de código fuente <building-from-source>` para su plataforma.

Guías de desarrollo
^^^^^^^^^^^^^^^^^^^

.. toctree::
   :titlesonly:
   :maxdepth: 1

   Contributing/Developer-Guide
   Contributing/Code-Style-Language-Versions
   Contributing/Quality-Guide
   Contributing/Migration-Guide
   Contributing/Build-Farms
   Contributing/Windows-Tips-and-Tricks
   Contributing/Contributing-To-ROS-2-Documentation

En que trabajar
^^^^^^^^^^^^^^^

Hemos identificado una serie de tareas en las que los miembros de la comunidad podrían trabajar: se pueden enumerar `buscando en los repositorios de ROS 2 problemas etiquetados como "help wanted" <https://github.com/search?q=user%3Aament+user%3Aros2+is%3Aopen+label%3A"help+wanted"&type=Issues>`__.
Si ve algo en esa lista en lo que le gustaría trabajar, comente el elemento para que otros sepan que lo esta trabajando.

También tenemos una etiqueta para los issues que creemos que deberían ser más accesibles para quienes contribuyen por primera vez, `etiquetados como “good first issue” <https://github.com/search?q=user%3Aament+user%3Aros2+is%3Aopen+label%3A%22good+first+issue%22&type=Issues>`__.
Si estas interesado en contribuir al proyecto ROS 2, te animamos a que primero eches un vistazo a estos issues.
Si deseas tener en cuenta un rango más amplio, agradecemos las contribuciones a cualquier issue abierto (u otros que puedas proponer), en particular las tareas que tienen un hito (milestone) lo cual significa que se espera incorporarlo en la próxima versión de ROS 2 (el hito será el próxima versión, por ejemplo, 'crystal').

Si tienes alguna contribución de código que solucione un error o mejore la documentación, envíalo como un pull request al repositorio correspondiente.
Para cambios más grandes, es una buena idea discutir la propuesta `en el foro de ROS 2 <https://discourse.ros.org/c/ng-ros>`__ antes de comenzar a trabajar en ella para que puedas identificar si alguien más ya está trabajando en algo similar.
Si tu propuesta implica cambios en la API, se recomienda encarecidamente que discutas el enfoque antes de comenzar a trabajar.

Presentando tus cambios al código
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Las contribuciones de código deben realizarse a través de pull rquest a `los repositorios ros2 apropiados <https://github.com/ros2>`__.

Pedimos a todos los colaboradores que sigan las prácticas explicadas en :doc:`la guía para desarrolladores <Contributing/Developer-Guide>`.

Asegúrate de :ref:`ejecutar pruebas <colcon-run-the-tests>` a el código que tiene tus cambios porque la mayoría de los paquetes tienen pruebas que verifican que el código cumple con nuestras pautas de estilo.

Convertirse en un core maintainer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Los maintainer de ROS 2 aseguran que el proyecto se mantenga progresando.
Las responsabilidades de los mantenedores incluyen:

* Revisar las contribuciones del código entrante en cuanto a estilo, calidad y en general alineamiento respecto a los objetivos del repositorio/ROS 2.
* Asegurar que CI continúe siendo exitoso.
* Incorporar los pull request que cumplan con los estándares de calidad y CI anteriores.
* Abordar problemas abiertos por los usuarios.

Cada repositorio en las organizaciones `ros2 <https://github.com/ros2>`__ y `ament <https://github.com/ament>`__ tiene un conjunto independiente de maintainers.
Convertirse en un maintainer de uno o más de esos repositorios es un proceso solo por invitación y generalmente implica los siguientes pasos:

* En el último año, tener un número sustancial de contribuciones al repositorio.
* En el último año, realice una cantidad considerable de revisiones a las pull request entrantes al repositorio.

Aproximadamente cada 3 meses, el equipo de ROS 2 revisará las contribuciones en todos los repositorios y enviará invitaciones a nuevos mantenedores.
Una vez que se acepta la invitación, se le pedirá al nuevo mantenedor que realice un breve proceso de capacitación sobre los mecanismos y políticas de los repositorios de ROS 2.
Después de que se complete ese proceso de capacitación, el nuevo mantenedor tendrá acceso de escritura a los repositorios apropiados.