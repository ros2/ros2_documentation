Complementos de seguridad de Connext
====================================

Las bibliotecas Connext DDS se incluyen con ROS 2 bajo una `licencia no comercial <https://www.rti.com/ncl>`__ 
y no incluyen las bibliotecas de complementos de seguridad. Estas bibliotecas están disponibles en las 
versiones de licencia comercial, universitaria y de investigación de RTI Connext DDS Pro, que incluye 
herramientas para la depuración, supervisión, grabación/reproducción, etc.

Se incluye un recorrido en video de esta instalación (herramientas y complementos de seguridad). disponible 
`aquí <https://www.rti.com/gettingstarted/installwindows_secure>`__ en el sitio web de RTI. Los pasos son:

**Instalar Connext DDS Pro (Host)**
Esta es una aplicación de instalación específica del host (para Windows, Linux, MacOS) para instalar un paquete 'Host' que incluye el Iniciador, las herramientas y otros servicios de software.
Al final de la instalación, se iniciará el programa RTI 'Launcher'.
El Lanzador se utiliza para instalar bibliotecas de destino, complementos de seguridad y otros servicios en capas.

**Utilizar el instalador de paquetes en Launcher**

.. figure:: https://cdn2.hubspot.net/hub/1754418/file-3649043118-png/blog-files/launchermacos.png
   :alt: Imagen del lanzador

   Imagen del lanzador

El 'Instalador de paquetes RTI' se usa para instalar archivos '.rtipkg': 
bibliotecas de destino, complementos de seguridad, etc. Abre el Instalador de paquetes y selecciona 
todos los archivos .rtipkg que se incluyeron en el paquete Connext DDS Secure para la instalación:


 * Bibliotecas de destino - como: rti\_connext\_dds-[version]-pro-target-[toolchain].rtipkg
 * Security Plugin Host - como: rti\_security\_plugins-[version]-host-[toolchain].rtipkg
 * Security Plugin Target - como: rti\_security\_plugins-[version]-target-[toolchain].rtipkg
 * OpenSSL Host - como: openssl-1.0.2x-[version]-host-[toolchain].rtipkg

**Extraer e instalar OpenSSL**
Esto se incluye como un archivo (.zip o de otro modo) y puede extraerse 
y copiarse fácilmente en una ubicación conveniente en su computadora host. 
Como sugerencia, esto también podría instalarse en el directorio 'rti\_connext\_dds-[version]' 
en su espacio de directorio de inicio (esto se creó durante la instalación de las herramientas de 
host RTI). Nota: es posible que debas colocar esta ubicación de directorio en su variable de entorno PATH.
Consulte la `Guía de introducción a los complementos de seguridad de RTI <https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/manuals/connext_dds/dds_security/RTI_SecurityPlugins_GettingStarted.pdf>`__ para obtener más información.

Instalación completa.
