.. redirect-from::

   Installation/Windows-Development-Setup

.. _windows-latest:

Windows (fuentes)
=================

.. contents:: Tabla de Contenidos
   :depth: 2
   :local:

Esta guía trata sobre cómo configurar un entorno de desarrollo para ROS 2 en Windows.

Requisitos del sistema
----------------------

Sólo Windows 10 está soportado.

Soporte de idioma support
^^^^^^^^^^^^^^^^^^^^^^^^^

Asegúrate de tener una configuración regional que admita ``UTF-8``.
Por ejemplo, para una instalación de Windows 10 en idioma chino, es posible que debas instalar un `paquete de idioma de inglés <https://support.microsoft.com/en-us/windows/language-packs-for-windows-a5094319-a92d-18de-5b53-1cfc697cfca8>`_.

.. include:: ../_Windows-Install-Prerequisites.rst

Prerequisitos adicionales
-------------------------

Al compilar desde el código fuente, necesitarás algunos prerequisitos adicionales instalados.

Instalar prerequisitos adicionales de Chocolatey
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   choco install -y cppcheck curl git winflexbison3

Deberás agregar la carpeta Git cmd ``C:\Program Files\Git\cmd`` al PATH (puedes hacerlo haciendo clic en el ícono de Windows, escribiendo "Variables de entorno" y luego haciendo clic en "Editar el entorno del sistema variables".
En el cuadro de diálogo resultante, haz clic en "Variables de entorno", haz clic en "Path" en el panel inferior, luego haz clic en "Editar" y agrega la ruta).


Instalar prerequisitos de Python
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Instalación de dependencias adicionales de Python:

.. code-block:: bash

   pip install -U colcon-common-extensions coverage flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes mock mypy==0.931 pep8 pydocstyle pytest pytest-mock vcstool

Instalar otros varios prerequisitos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A continuación instala xmllint:

* Descarga los `archivos binarios de 64 bits <https://www.zlatkovic.com/pub/libxml/64bit/>`__ de ``libxml2`` (y sus dependencias ``iconv`` y ``zlib``) de https://www.zlatkovic.com/projects/libxml/
* Descomprime todos los archivos en, por ejemplo, ``C:\xmllint``
* Agrega ``C:\xmllint\bin`` al ``PATH``.

Obtener el código ROS 2
-----------------------

Ahora que tenemos las herramientas de desarrollo podemos obtener el código fuente de ROS 2.

Primero configura una carpeta de desarrollo, por ejemplo ``C:\{DISTRO}``:

.. note::

   Es muy importante que la ruta elegida sea corta, debido a los cortos límites predeterminados de ruta de Windows (260 caracteres).
   Para permitir rutas más largas, consulta https://learn.microsoft.com/en-us/windows/win32/fileio/maximum-file-path-limitation?tabs=registry.

.. code-block:: bash

   md \{DISTRO}\src
   cd \{DISTRO}

Obtén el archivo ``ros2.repos`` que define los repositorios desde los que clonar:

.. code-block:: bash

   vcs import --input https://raw.githubusercontent.com/ros2/ros2/{REPOS_FILE_BRANCH}/ros2.repos src

Instalar implementaciones de DDS adicionales (opcional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Fast DDS se incluye con las fuentes de ROS 2 y siempre se compilará a menos que coloque un archivo ``COLCON_IGNORE`` en la carpeta ``src\eProsima``.

Si deseas utilizar otro proveedor de DDS o RTPS además del predeterminado, puedes encontrar instrucciones :doc:`aquí <../DDS-Implementations>`.

Compila el código ROS 2
-------------------------

.. _windows-dev-build-ros2:

Para compilar ROS 2, necesitarás un símbolo del sistema de Visual Studio ("x64 Native Tools Command Prompt for VS 2019") ejecutándose como administrador.

Para compilar el árbol de la carpeta ``\{DISTRO}``:

.. code-block:: bash

   colcon build --merge-install

.. note::

   Estamos usando ``--merge-install`` aquí para evitar una variable ``PATH`` que sea demasiado larga al final de la compilación.
   Si estás adaptando estas instrucciones para crear un espacio de trabajo más pequeño, es posible que pueda usar el comportamiento predeterminado que es una instalación aislada, es decir, donde cada paquete se instala en una carpeta diferente.

.. note::

   Si estás haciendo una compilación de depuración, usa ``python_d path\to\colcon_executable`` ``colcon``.
   Consulta `Cosas adicionales para el modo de depuración`_ para obtener más información sobre cómo ejecutar el código de Python en compilaciones de depuración en Windows.

Configuración del entorno
-------------------------

Inicia una shell de comandos y obtén el archivo de configuración de ROS 2 para configurar el espacio de trabajo:

.. code-block:: bash

   call C:\{DISTRO}\install\local_setup.bat

Esto configurará automáticamente el entorno para cualquier proveedor de DDS para el soporte se haya compilado.

Es normal que el comando anterior, si nada más salió mal, muestre "El sistema no puede encontrar la ruta especificada." exactamente una vez.

Probar y ejecutar
-----------------

Tenga en cuenta que la primera vez que ejecutes cualquier ejecutable, deberás permitir el acceso a la red a través de una ventana emergente del Firewall de Windows.

Puedes ejecutar las pruebas usando este comando:

.. code-block:: bash

   colcon test --merge-install

.. note::

   ``--merge-install`` solo debe usarse si también se usó en el paso de compilación.

Luego puedes obtener un resumen de las pruebas usando este comando:

.. code-block:: bash

   colcon test-result

Para ejecutar los ejemplos, primero abre un archivo ``cmd.exe`` nuevo y limpio y configura el espacio de trabajo ejecutando el archivo ``local_setup.bat``.
Luego, ejecuta un C++ ``talker``\ :

.. code-block:: bash

   call install\local_setup.bat
   ros2 run demo_nodes_cpp talker

En un shell separado, puedes hacer lo mismo, pero en su lugar ejecute Python ``listener``\ :

.. code-block:: bash

   call install\local_setup.bat
   ros2 run demo_nodes_py listener

Deberías ver al ``talker`` diciendo que está publicando (``Publishing``) mensajes y al ``listener`` diciendo que oye (``I heard``) esos mensajes.
Esto verifica que las API de C++ y Python funcionan correctamente.
¡Hurra!


.. note::

   No se recomienda compilar en el mismo cmd prompt en el que ejecutó ``local_setup.bat``.

Siguientes pasos después de la instalación
------------------------------------------
Continúa con los :doc:`tutoriales y demostraciones <../../Tutorials>` para configurar tu entorno, crear tu propio espacio de trabajo y paquetes, y aprender los conceptos básicos de ROS 2.

Implementaciones adicionales de RMW (opcional)
----------------------------------------------
El middleware predeterminado que usa ROS 2 es ``Fast DDS``, pero el middleware (RMW) se puede reemplazar en tiempo de ejecución.
Consultea la :doc:`guía <../../How-To-Guides/Working-with-multiple-RMW-implementations>` sobre cómo trabajar con múltiples RMW.


Cosas adicionales para el modo de depuración
--------------------------------------------

Si deseas poder ejecutar todas las pruebas en modo de depuración, deberás instalar algunas cosas más:


* Para poder extraer el tarball fuente de Python, puedes usar PeaZip:

.. code-block:: bash

   choco install -y peazip


* También necesitarás SVN, ya que algunas de las dependencias de compilación de fuentes de Python se verifican a través de SVN:

.. code-block:: bash

   choco install -y svn hg

* Deberás salir y reiniciar el símbolo del sistema después de instalar lo anterior.
* Obtén y extráe las fuentes de Python 3.8.3 del ``tgz``:

  * https://www.python.org/ftp/python/3.8.3/Python-3.8.3.tgz
  * Para mantener estas instrucciones concisas, extráelas a ``C:\dev\Python-3.8.3``

* Ahora, copila lo fuentes de Python en modo de depuración desde un símbolo del sistema de Visual Studio:

.. code-block:: bash

   cd C:\dev\Python-3.8.3\PCbuild
   get_externals.bat
   build.bat -p x64 -d


* Finalmente, copia los productos de compilación en los directorios de instalación de Python38, junto al ejecutable de Python en modo release y las DLL's:

.. code-block:: bash

   cd C:\dev\Python-3.8.3\PCbuild\amd64
   copy python_d.exe C:\Python38 /Y
   copy python38_d.dll C:\Python38 /Y
   copy python3_d.dll C:\Python38 /Y
   copy python38_d.lib C:\Python38\libs /Y
   copy python3_d.lib C:\Python38\libs /Y
   copy sqlite3_d.dll C:\Python38\DLLs /Y
   for %I in (*_d.pyd) do copy %I C:\Python38\DLLs /Y


* Ahora, desde un nuevo símbolo del sistema, asegúrate de que ``python_d`` funciona:

.. code-block:: bash

   python_d -c "import _ctypes ; import coverage"

* Una vez que hayas verificado el funcionamiento de ``python_d``, es necesario reinstalar algunas dependencias con las bibliotecas habilitadas para depuración:

.. code-block:: bash

   python_d -m pip install --force-reinstall https://github.com/ros2/ros2/releases/download/numpy-archives/numpy-1.18.4-cp38-cp38d-win_amd64.whl
   python_d -m pip install --force-reinstall https://github.com/ros2/ros2/releases/download/lxml-archives/lxml-4.5.1-cp38-cp38d-win_amd64.whl

* Para verificar la instalación de estas dependencias:

.. code-block:: bash

   python_d -c "from lxml import etree ; import numpy"

* Cuando desees volver a compilar los binarios de "release", es necesario desinstalar las variantes de depuración y utilizar la de versión "release":

.. code-block:: bash

   python -m pip uninstall numpy lxml
   python -m pip install numpy lxml

* Para crear scripts ejecutables de python (.exe), se debe usar python_d para invocar a colcon

.. code-block:: bash

   python_d path\to\colcon_executable build

* ¡Hurra, ya terminaste!

Estar al día
------------

Consulta :doc:`../Maintaining-a-Source-Checkout` para actualizar periódicamente la instalación de fuentes.

Solución de problemas
---------------------

Las técnicas de solución de problemas se pueden encontrar :ref:`aquí <windows-troubleshooting>`.

Uninstall
---------

1. Si instalaste tu espacio de trabajo con colcon como se indicó anteriormente, la "desinstalación" podría ser simplemente una cuestión de abrir una nueva terminal y no ejecutar el archivo de ``setup`` del espacio de trabajo.
   De esta manera, tu entorno se comportará como si no hubiera una instalación de {DISTRO_TITLE} en su sistema.

2. Si también estás intentando liberar espacio, puedes eliminar todo el directorio del espacio de trabajo con:

   .. code-block:: bash

      rmdir /s /q \ros2_{DISTRO}
