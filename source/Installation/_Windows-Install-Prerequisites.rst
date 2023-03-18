Instalando prerequisitos
------------------------

Instalar Chocolatey
^^^^^^^^^^^^^^^^^^^

Chocolatey es un administrador de paquetes para Windows, instálalo siguiendo sus instrucciones de instalación:

https://chocolatey.org/

Usarás Chocolatey para instalar otras herramientas de desarrollo.

Instalar Python
^^^^^^^^^^^^^^^

Abre un símbolo del sistema y escriba lo siguiente para instalar Python a través de Chocolatey:

.. code-block:: bash

   choco install -y python --version 3.8.3

Instalar redistribuibles de Visual C++
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Abre un símbolo del sistema y escriba lo siguiente para instalarlos a través de Chocolatey:

.. code-block:: bash

   choco install -y vcredist2013 vcredist140

Instalar OpenSSL
^^^^^^^^^^^^^^^^

Descarga el instalador OpenSSL de *Win64 OpenSSL v1.1.1n* desde `esta página <https://slproweb.com/products/Win32OpenSSL.html>`__.
Desplázate hasta la parte inferior de la página y descarga *Win64 OpenSSL v1.1.1n*.
No descargues las versiones Win32 o Light, o los instaladores v3.X.Y.

Ejecuta el instalador con los parámetros predeterminados, ya que los siguientes comandos asumen que usaste el directorio de instalación predeterminado.

Este comando establece una variable de entorno que persiste durante las sesiones:

.. code-block:: bash

   setx /m OPENSSL_CONF "C:\Program Files\OpenSSL-Win64\bin\openssl.cfg"

Deberá agregar la carpeta bin OpenSSL-Win64 a su PATH.
Puede hacerlo haciendo clic en el icono de Windows, escribiendo "Variables de entorno" y luego haciendo clic en "Editar las variables de entorno del sistema".
En el cuadro de diálogo resultante, haga clic en "Variables de entorno", luego haga clic en "Path" en el panel inferior, finalmente haga clic en "Editar" y agregue la ruta a continuación.

* ``C:\Program Files\OpenSSL-Win64\bin\``

Instalar Visual Studio
^^^^^^^^^^^^^^^^^^^^^^

Instala Visual Studio 2019.

Si ya tienes una versión de pago de Visual Studio 2019 (Professional, Enterprise), omite este paso.

Microsoft proporciona una versión gratuita de Visual Studio 2019, llamada Community, que se puede usar para crear aplicaciones que usan ROS 2.
`You can download the installer directly through this link. <https://visualstudio.microsoft.com/thank-you-downloading-visual-studio/?sku=Community&rel=16&src=myvs&utm_medium=microsoft&utm_source=my.visualstudio.com&utm_campaign=download&utm_content=vs+community+2019>`_

Asegúrate de que las características de Visual C++ estén instaladas.

Una manera fácil de asegurarse de que estén instalados es seleccionar el flujo de trabajo ``Desarrollo de escritorio con C++`` durante la instalación.

   .. image:: /Installation/images/windows-vs-studio-install.png

Asegúrate de que no haya instaladas herramientas de CMake de C++ deseleccionándolas en la lista de componentes que se instalarán.

Instalar OpenCV
^^^^^^^^^^^^^^^

Algunos de los ejemplos requieren la instalación de OpenCV.

Puedes descargar una versión precompilada de OpenCV 3.4.6 desde https://github.com/ros2/ros2/releases/download/opencv-archives/opencv-3.4.6-vc16.VS2019.zip .

Suponiendo que lo descomprimiste en ``C:\opencv``, escribe lo siguiente en un símbolo del sistema (requiere privilegios de administrador):

.. code-block:: bash

   setx /m OpenCV_DIR C:\opencv

Dado que está utilizando una versión ROS precompilada, debemos indicarle dónde encontrar las bibliotecas de OpenCV.
Tienes que extender la variable ``PATH`` a ``C:\opencv\x64\vc16\bin``.

Instalar dependencias
^^^^^^^^^^^^^^^^^^^^^

Hay algunas dependencias que no están disponibles en la base de datos del paquete Chocolatey.
Para facilitar el proceso de instalación manual, proporcionamos los paquetes Chocolatey necesarios.

Como algunos paquetes chocolatosos dependen de él, comenzamos instalando CMake

.. code-block:: bash

   choco install -y cmake

Deberás agregar la carpeta bin de CMake ``C:\Program Files\CMake\bin`` a tu PATH.

Descarga estos paquetes desde `este <https://github.com/ros2/choco-packages/releases/latest>`__ repositorio de GitHub.

* asio.1.12.1.nupkg
* bullet.3.17.nupkg
* cunit.2.1.3.nupkg
* eigen-3.3.4.nupkg
* tinyxml-usestl.2.6.2.nupkg
* tinyxml2.6.0.0.nupkg

Una vez que se descargan estos paquetes, abre un shell administrativo y ejecuta el siguiente comando:

.. code-block:: bash

   choco install -y -s <PATH\TO\DOWNLOADS\> asio cunit eigen tinyxml-usestl tinyxml2 bullet

Reemplace ``<PATH\TO\DOWNLOADS>`` con la carpeta en la que descargó los paquetes.

Primero actualiza pip y setuptools:

.. code-block:: bash

   python -m pip install -U pip setuptools==59.6.0

Ahora instala algunas dependencias de Python adicionales:

.. code-block:: bash

   python -m pip install -U catkin_pkg cryptography empy importlib-metadata jsonschema lark==1.1.1 lxml matplotlib netifaces numpy opencv-python PyQt5 pillow psutil pycairo pydot pyparsing==2.4.7 pyyaml rosdistro

Instalar Qt5
^^^^^^^^^^^^

Descarga el `instalador offline 5.12.X <https://www.qt.io/offline-installers>`_ del sitio web de Qt.
Ejecuta el instalador.
Asegúrate de seleccionar el componente ``MSVC 2017 64-bit`` en el árbol ``Qt`` -> ``Qt 5.12.12``.

Finalmente, en una ventana de administrador ``cmd.exe`` configura estas variables de entorno.
Los siguientes comandos asumen que lo instalaste en la ubicación predeterminada de ``C:\Qt``.

.. code-block:: bash

   setx /m Qt5_DIR C:\Qt\Qt5.12.12\5.12.12\msvc2017_64
   setx /m QT_QPA_PLATFORM_PLUGIN_PATH C:\Qt\Qt5.12.12\5.12.12\msvc2017_64\plugins\platforms


.. note::

   Esta ruta puede cambiar según la versión de MSVC instalada, el directorio en el que se instaló Qt y la versión de Qt instalada.

Dependencias de RQt
^^^^^^^^^^^^^^^^^^^

Para ejecutar rqt_graph, debe `descargar <https://graphviz.gitlab.io/_pages/Download/Download_windows.html>`__ e instalar `Graphviz <https://graphviz.gitlab.io/>`__.
El instalador le preguntará si desea agregar graphviz a PATH, elige agregarlo al usuario actual o a todos los usuarios.

Instalar implementaciones de DDS adicionales (opcional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Si desea utilizar otro proveedor de DDS o RTPS además del DDS rápido predeterminado, puede encontrar instrucciones `aquí </Installation/DDS-Implementations>`_.
