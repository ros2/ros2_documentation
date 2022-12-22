.. _InstallationGuide:
.. _RollingInstall:

Instalación
===========

Opciones para instalar ROS 2 {DISTRO_TITLE_FULL}:

.. toctree::
   :hidden:
   :glob:

   Installation/Ubuntu-Install-Debians
   Installation/Windows-Install-Binary
   Installation/RHEL-Install-RPMs
   Installation/Alternatives
   Installation/Maintaining-a-Source-Checkout
   Installation/Testing
   Installation/DDS-Implementations

Paquetes binarios
---------------

Los archivos binarios solo se crean para los sistemas operativos Tier 1 enumerados en `REP-2000 <https://www.ros.org/reps/rep-2000.html#rolling-ridley-june-2020-ongoing>`__.
Dada la naturaleza de Rolling, esta lista podrá ser actualizada en cualquier momento.
Si no estás ejecutando ninguno de los siguientes sistemas operativos, es posible que debas compilar desde la fuente o usar :doc:`contenedor <How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers>` para ejecutar ROS 2 en tu plataforma.

Proporcionamos paquetes binarios de ROS 2 para las siguientes plataformas:

* Ubuntu Linux - Jammy Jellyfish (22.04)

  * :doc:`Paquetes Debian <Installation/Ubuntu-Install-Debians>` (recomendedado)
  * :doc:`Archivo "fat" <Installation/Alternatives/Ubuntu-Install-Binary>`

* RHEL 8

  * :doc:`Paquetes RPM <Installation/RHEL-Install-RPMs>` (recomendedado)
  * :doc:`Archivo "fat" <Installation/Alternatives/RHEL-Install-Binary>`

* :doc:`Windows (VS 2019) <Installation/Windows-Install-Binary>`


.. _building-from-source:

Construyendo desde la fuente
--------------------

Apoyamos la creación de ROS 2 desde fuentes en las siguientes plataformas:


* :doc:`Ubuntu Linux <Installation/Alternatives/Ubuntu-Development-Setup>`
* :doc:`Windows <Installation/Alternatives/Windows-Development-Setup>`
* :doc:`RHEL <Installation/Alternatives/RHEL-Development-Setup>`
* :doc:`macOS <Installation/Alternatives/macOS-Development-Setup>`


¿Qué instalación elegir?
------------------------

La instalación desde paquetes binarios o desde fuentes dará como resultado una instalación de ROS 2 totalmente funcional y utilizable.
Las diferencias entre las opciones dependen de lo que planee hacer con ROS 2.

**Los paquetes binarios** son para uso general y proporcionan una instalación ya compilada de ROS 2.
Esto es excelente para las personas que desean sumergirse y comenzar a usar ROS 2 tal como está, de inmediato.

Los usuarios de Linux tienen dos opciones para instalar paquetes binarios:

- Paquetes Debian
- Archivo "fat"

La instalación desde paquetes Debian es el método recomendado.
Es más conveniente porque instala automáticamente las dependencias necesarias.
También se actualiza junto con las actualizaciones periódicas del sistema.

Sin embargo, necesita acceso de root para instalar los paquetes de Debian.
Si no tiene acceso de root, el archivo "fat" es la siguiente mejor opción.

Los usuarios de macOS y Windows que eligen instalar desde paquetes binarios solo tienen la opción de archivo "fat"
(Los paquetes de Debian son exclusivos de Ubuntu/Debian).

**Crear desde fuentes** está destinado a los desarrolladores que buscan alterar u omitir explícitamente partes de la base de ROS 2.
También se recomienda para plataformas que no admiten binarios.
Construir desde fuentes también le da la opción de instalar la última versión absoluta de ROS 2.

¿Contribuyendo al núcleo de ROS 2?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Si planeas contribuir directamente a los paquetes principales de ROS 2, puedes instalar el :doc:`último desarrollo de fuentes <Instalación/Alternativas/Latest-Development-Setup>` que comparte las instrucciones de instalación con la :ref:`Distribución Rolling <rolling_distribution>`.
