implementaciones de DDS
=======================

De forma predeterminada, ROS 2 usa DDS como `middleware <https://design.ros2.org/articles/ros_on_dds.html>`__.
Es compatible con varios proveedores de DDS o RTPS (el protocolo de conexión DDS).
Actualmente hay soporte para Fast DDS de eProsima, Connext DDS de RTI, Eclipse Cyclone DDS y GurumNetworks GurumDDS.
Consulta https://ros.org/reps/rep-2000.html para conocer los proveedores de DDS soportados por distribución.

En Rolling, el proveedor de DDS predeterminado es Fast DDS de eProsima.

* :doc:`Trabajando con Eclipse Cyclone DDS <DDS-Implementations/Working-with-Eclipse-CycloneDDS>` explica cómo utilizar Cyclone DDS.
* :doc:`Trabajando con eProsima Fast DDS <DDS-Implementations/Working-with-eProsima-Fast-DDS>` explica cómo utilizar Fast DDS.
* :doc:`Trabajando con GurumNetworks GurumDDS <DDS-Implementations/Working-with-GurumNetworks-GurumDDS>` explica cómo utilizar GurumDDS.

.. toctree::
   :hidden:
   :glob:

   DDS-Implementations/*

Si deseas utilizar uno de los otros proveedores, debes instalar su software por separado antes de compilar.
La compilación de ROS 2 creará soporte automáticamente para los proveedores que se hayan instalado y activado correctamente.

Una vez que hayas instalado un nuevo proveedor de DDS, puedes cambiar el proveedor utilizado en el tiempo de ejecución: :doc:`Trabajar con múltiples implementaciones de RMW <../How-To-Guides/Trabajar-con-varias-implementaciones-RMW>`.

A continuación se proporcionan instrucciones detalladas para instalar otros proveedores de DDS.

.. contents:: Platforms / Installation types
   :depth: 1
   :local:

Instalación de Ubuntu Linux de fuentes
--------------------------------------

RTI Connext (versión 6.0.1, solo amd64)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Paquetes Debian provistos en los repositorios apt de ROS 2
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Puedes instalar un paquete Debian de RTI Connext disponible en los repositorios apt de ROS 2.
Deberás aceptar una licencia de RTI.

.. code-block:: bash

   sudo apt update && sudo apt install -q -y rti-connext-dds-6.0.1

Ejecuta `source` sobre el fichero de configuraciónpara establecer la variable de entorno ``NDDSHOME``.

.. code-block:: bash

   cd /opt/rti.com/rti_connext_dds-6.0.1/resource/scripts && source ./rtisetenv_x64Linux4gcc7.3.0.bash; cd -

Nota: al usar ``zsh``, debe estar en el directorio del script al ejecutar `source` para que funcione correctamente.

Ahora puedes compilar con normalidad y también se compilará soporte para RTI.

Paquetes binarios oficiales de RTI
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Puedes instalar el paquete Connext 6.0.1 para Linux provisto por RTI, a través de las opciones disponibles para :doc:`universidad, compra o evaluación <DDS-Implementations/Install-Connext-University-Eval>`

Después de descargar, usa ``chmod +x`` en el ejecutable ``.run`` y luego ejecútalo.
Ten en cuenta que si está instalando en un directorio del sistema, usa además ``sudo``.

La ubicación predeterminada es ``~/rti_connext_dds-6.0.1``

Después de la instalación, ejecuta el launcher de RTI y diríjelo a tu archivo de licencia (obtenido de RTI).

Agrega la siguiente línea a su archivo ``.bashrc`` apuntando a tu copia de la licencia.

.. code-block:: bash

   export RTI_LICENSE_FILE=path/to/rti_license.dat

Ejecuta `source` con el archivo de configuración para configurar la variable de entorno ``NDDSHOME``.

.. code-block:: bash

   cd ~/rti_connext_dds-6.0.1/resource/scripts && source ./rtisetenv_x64Linux4gcc7.3.0.bash; cd -

Ahora puedes compilar con normalidad y también se compilará soporte para RTI.

Instalación de binarios en Ubuntu Linux
---------------------------------------

RTI Connext (versión 6.0.1, solo amd64)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Para usar RTI Connext DDS, hay opciones de instalación de conjunto completo disponibles para :doc:`universidad, compra o evaluación <DDS-Implementations/Install-Connext-University-Eval>`
o puedes instalar un paquete Debian con únicamente las bibliotecas de RTI Connext 6.0.1, disponible en el repositorio OSRF Apt
bajo una `licencia no comercial <https://www.rti.com/ncl>`__.

Para instalar el paquete Debian únicamente con bibliotecas:

.. code-block:: bash

   sudo apt update && sudo apt install -q -y rti-connext-dds-6.0.1

Deberás aceptar un acuerdo de licencia de RTI y encontrar un archivo 'rti_license.dat' en la instalación.

Agrega la siguiente línea a su archivo ``.bashrc`` apuntando a tu copia de la licencia (y ejecyta `source`).

.. code-block:: bash

   export RTI_LICENSE_FILE=path/to/rti_license.dat

Todas las opciones necesitan que ejecutes `source` son el archivo de configuración para establecer la variable de entorno ``NDDSHOME``:

.. code-block:: bash

   cd /opt/rti.com/rti_connext_dds-6.0.1/resource/scripts && source ./rtisetenv_x64Linux4gcc7.3.0.bash; cd -

Nota: lo anterior puede necesitar modificaciones para que coincida con su ubicación de instalación de RTI

Si deseas instalar los complementos de Connext DDS-Security, consulta :doc:`esta página <DDS-Implementations/Install-Connext-Security-Plugins>`.

Instación de fuentes en OSX
---------------------------

RTI Connext (6.0.1)
^^^^^^^^^^^^^^^^^^^

Si deseas compilar también con RTI Connext DDS, hay opciones disponibles para :doc:`universidad, compra o evaluación <DDS-Implementations/Install-Connext-University-Eval>`

También necesitas un entorno de ejecución Java instalado para ejecutar el generador de código RTI, que puedes obtener `aquí <https://support.apple.com/kb/DL1572?locale=en_US>`__.

Después de la instalación, ejecuta el launcher de RTI y diríjelo a tu archivo de licencia.

Ejecuta `source` con el archivo de configuración para configurar la variable de entorno ``NDDSHOME`` antes de compilar tu workspace.

.. code-block:: bash

   source /Applications/rti_connext_dds-6.0.1/resource/scripts/rtisetenv_x64Darwin17clang9.0.bash

Es posible que debas aumentar los recursos de memoria compartida siguiendo https://community.rti.com/kb/osx510

Si deseas instalar los complementos de Connext DDS-Security, consulta :doc:`esta página <DDS-Implementations/Install-Connext-Security-Plugins>`.

Instalación con binarios en OSX
-------------------------------


Habilitar el soporte de Connext
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Para usar RTI Connext DDS hay opciones disponibles para :doc:`universidad, compra o evaluación <DDS-Implementations/Install-Connext-University-Eval>`

Después de la instalación, ejecuta el launcher de RTI y diríjelo a tu archivo de licencia.

Ejecuta `source` con el archivo de configuración para configurar la variable de entorno ``NDDSHOME`` antes de compilar tu workspace.

.. code-block:: bash

   source /Applications/rti_connext_dds-6.0.1/resource/scripts/rtisetenv_x64Darwin17clang9.0.bash

Es posible que debas aumentar los recursos de memoria compartida siguiendo https://community.rti.com/kb/osx510

Si deseas instalar los complementos de Connext DDS-Security, consulta :doc:`esta página <DDS-Implementations/Install-Connext-Security-Plugins>`.

Windows source install
----------------------

RTI Connext 6.0.1
^^^^^^^^^^^^^^^^^

Si deseas compilar también con RTI Connext DDS, hay opciones disponibles para :doc:`universidad, compra o evaluación <DDS-Implementations/Install-Connext-University-Eval>`

Después de la instalación, ejecuta el launcher de RTI y diríjelo a tu archivo de licencia.

Luego, antes de compilar ROS 2, configura el entorno Connext:

.. code-block:: bash

   call "C:\Program Files\rti_connext_dds-6.0.1\resource\scripts\rtisetenv_x64Win64VS2017.bat"

Tenga en cuenta que es posible que esta ruta deba modificarse ligeramente en función de dónde seleccionaste instalar RTI Connext DDS y qué versión de Visual Studio se seleccionó.
La ruta anterior es la ruta predeterminada actual a partir de la versión 6.0.1, pero cambiará a medida que aumenten los números de versión en el futuro.

Si deseas instalar los complementos de Connext DDS-Security, consultea :doc:`esta página <DDS-Implementations/Install-Connext-Security-Plugins>`.

Instalción de binarios en Windows
---------------------------------


RTI Connext
^^^^^^^^^^^

Para usar RTI Connext DDS hay opciones disponibles para :doc:`universidad, compra o evaluación <DDS-Implementations/Install-Connext-University-Eval>`

Después de la instalación, ejecuta el launcher de RTI y diríjelo a tu archivo de licencia.

Luego, antes de usar ROS 2, configura el entorno Connext:

.. code-block:: bash

   call "C:\Program Files\rti_connext_dds-6.0.1\resource\scripts\rtisetenv_x64Win64VS2017.bat"

Si deseas instalar los complementos de Connext DDS-Security, consultea :doc:`esta página <DDS-Implementations/Install-Connext-Security-Plugins>`.
