.. redirect-from::

    Contributing/Contributing-To-ROS-2-Documentation

Contribuir a la Documentación de  ROS 2
=======================================

.. contents:: Tabla de Contenido
   :depth: 2
   :local:

Las contribuciones a este sitio son bienvenidas.
Esta página explica cómo contribuir a la documentación de ROS 2.
Asegúrate de leer detenidamente las siguientes secciones antes de contribuir.

El sitio está construido usando  `Sphinx <https://www.sphinx-doc.org/en/master/>`__, especificamente `Sphinx multiversion <https://holzhaus.github.io/sphinx-multiversion/master/index.html>`__.

Estructura de la rama
---------------------

El código fuente de la documentación se encuentra en `el repositorio de Github de ROS 2 Documentation <https://github.com/ros2/ros2_documentation>`_.
Este repositorio está configurado asignando una rama a cada distribución de ROS 2 para manejar las diferencias entre las distribuciones.
Si un cambio es común a todas las distribuciones de ROS 2, debe realizarse en la rama ``rolling`` (y luego se actualizará según corresponda).
Si un cambio es específico a una distribución particular de ROS 2, debe realizarse en la respectiva rama.

Estructura del código fuente
----------------------------

Los archivos fuente del sitio están todos ubicados en la carpeta ``source``.
Las plantillas para varios complementos de sphinx se encuentran en ``source/_templates``.
El directorio raíz del repositorio contiene la configuración y los archivos necesarios para construir localmente el sitio para hacer pruebas.

Construir el sitio localmente
-----------------------------

Comience instalando los requisitos ubicados en el archivo ``requirements.txt``:

.. tabs::

  .. group-tab:: Linux

     El siguiente comando realiza una instalación de tipo usuario actual, este requiere que se agregue ``~/.local/bin/`` a ``$PATH``:

   .. code-block:: console

         pip3 install --user --upgrade -r requirements.txt

  .. group-tab:: macOS

   .. code-block:: console

         pip3 install --user --upgrade -r requirements.txt

  .. group-tab:: Windows

    .. code-block:: console

      python -m pip install --user --upgrade -r requirements.txt

Para que Sphinx pueda generar diagramas, el comando ``dot`` debe estar disponible.

.. tabs::

  .. group-tab:: Linux

    .. code-block:: console

       sudo apt update ; sudo apt install graphviz

  .. group-tab:: macOS

    .. code-block:: console

      brew install graphviz

  .. group-tab:: Windows

      Descarga un instalador  de `la pagina de descargas de Graphviz <https://graphviz.gitlab.io/_pages/Download/Download_windows.html>`__ e instálalo.
      Asegurate de permitir que el instalador añada a Graphviz al ``%PATH%``de Windows de lo contrario Sphinx no lo encontrara.

Construir el sitio para una rama
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Para construir el sitio unicamente para la rama actual, escribe ``make html`` en la carpeta raíz del repositorio.
Esta es la forma recomendada de probar los cambios locales.

.. code-block:: console

   make html

El proceso de compilación puede llevar algún tiempo.
Para ver el resultado, abra ``build/html/index.html`` en su navegador.

También puede ejecutar las pruebas de documentación localmente (usando `doc8 <https://github.com/PyCQA/doc8>`_) con el siguiente comando:

.. code-block:: console

   make test

Construir el sitio para todas las ramas.
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Para construir el sitio para todas las ramas, escriba ``make multiversion`` desde la rama ``rolling``.
Esto tiene dos inconvenientes:

#. El complemento multiversión no entiende cómo hacer compilaciones incrementales, por lo que siempre reconstruye todo.
   Esto puede ser lento.

#. Al escribir ``make multiversion``, siempre verificará exactamente las ramas enumeradas en el archivo ``conf.py``.
   Eso significa que no se mostrarán los cambios locales.

Para mostrar los cambios locales en la salida de varias versiones, primero debes realizar un commit con los cambios en una rama local.
Luego debes editar el archivo `conf.py <https://github.com/ros2/ros2_documentation/blob/rolling/conf.py>`_ y cambiar la variable ``smv_branch_whitelist`` para que apunte a tu rama.

Comprobación de enlaces rotos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Para verificar si hay enlaces rotos en el sitio, ejecute:

.. code-block:: console

   make linkcheck

Esto comprobará todo el sitio en busca de enlaces rotos y mostrará los resultados en la pantalla y en ``build/linkcheck``.

Escribir paginas
----------------

El sitio web de documentación de ROS 2 utiliza el formato ``reStructuredText``, que es el lenguaje de markup de texto plano que Sphinx utiliza de manera predeterminada.
Esta sección es una breve introducción a los conceptos, la sintaxis y las mejores prácticas de ``reStructuredText``.

Puede consultar la `Documentación de reStructuredText  <https://docutils.sourceforge.io/rst.html>`_ para obtener una especificación técnica detallada.

Tabla de Contenido
^^^^^^^^^^^^^^^^^^

Hay dos tipos de directivas utilizadas para la generación de una tabla de contenido, ``.. toctree::`` y ``.. content::``.
El ``.. toctree::`` se usa en páginas de nivel superior como ``Tutorials.rst`` para establecer el orden y la visibilidad de sus páginas secundarias.
Esta directiva crea tanto el panel de navegación izquierdo como los enlaces de navegación en la página a las páginas secundarias enumeradas.
Ayuda a los lectores a comprender la estructura de diferentes secciones de la documentación y la navegación entre ellos.

.. code-block:: rst

   .. toctree::
      :maxdepth: 1

La directiva ``..contents::`` se usa para generar una tabla de contenido para esa página en particular.
Analiza todos los encabezados presentes en una página y crea una tabla de contenido dentro de la página.
Ayuda a los lectores a ver una descripción general del contenido y navegar dentro de una página.

La directiva ``..contents::`` soporta la definición de profundidad máxima de secciones anidadas.
Al usar ``: depth: 2`` solo mostrará Secciones y Subsecciones en la tabla de contenido.

.. code-block:: rst

   .. contents:: Tabla de Contenido
      :depth: 2
      :local:

Encabezados
^^^^^^^^^^^

Hay cuatro tipos principales de encabezados utilizados en la documentación.
Ten en cuenta que el número de símbolos tiene que coincidir con la longitud del título.

.. code-block:: rst

   Encabezado titulo de pagina
   ==========================

   Encabezado de Sección
   ---------------------

   2 Encabezado de Subsección
   ^^^^^^^^^^^^^^^^^^^^^^^^^^

   2.4 Encabezado de Subsubsección
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Por lo general, usamos un dígito para numerar las subsecciones y dos dígitos (separados por puntos) para numerar las subsubsecciones en los tutoriales y las How-To-Guides.

Listas
^^^^^^

Las estrellas ``*`` se usan para listar con viñetas elementos desordenados  y el signo de número ``#.`` se usa para listar elementos numerados.
Ambos admiten definiciones anidadas y se muestran de manera acorde.

.. code-block:: rst

   * viñeta

     * viñeta anidada
     * viñeta anidada

   * viñeta

.. code-block:: rst

  #. Pirmer elemento
  #. Segundo elemento

Formato de código
^^^^^^^^^^^^^^^^^
Las ``comillas invertidas`` pueden ser usadas para ``resaltar`` código en el texto.


.. code-block:: rst

   Las ``comillas invertidas`` pueden ser usadas para ``resaltar`` código en el texto.

Los bloques de código dentro de una página necesitan ser encerrados usando la directiva ``.. code-block::``.
``.. code-block::`` admite el resaltado de código para sintaxis como ``C++``, ``YAML``, ``console``, ``bash`` y más.
El código dentro de la directiva debe estar indentado.

.. code-block:: rst

   .. code-block:: C++

      int main(int argc, char** argv)
      {
         rclcpp::init(argc, argv);
         rclcpp::spin(std::make_shared<ParametersClass>());
         rclcpp::shutdown();
         return 0;
      }

Imágenes
^^^^^^^^

Las imágenes se pueden insertar usando la directiva ``.. image::``.

.. code-block:: rst

   .. image:: images/turtlesim_follow1.png

Referencias y enlaces
^^^^^^^^^^^^^^^^^^^^^

Enlaces externos
~~~~~~~~~~~~~~~~

La sintaxis para crear enlaces a páginas web externas se muestra a continuación.

.. code-block:: rst

    `Documentos de ROS <https://docs.ros.org>`_

El enlace anterior aparecerá como `Documentos de ROS <https://docs.ros.org>`_.
Tenga en cuenta el guión bajo después de la comilla simple final.

Enlaces internos
~~~~~~~~~~~~~~~~

La directiva ``:doc:`` se usa para crear enlaces en el texto a otras páginas.

.. code-block:: rst

    :doc:`Calidad de Servicio <../Tutorials/Quality-of-Service>`

Tenga en cuenta que se utiliza la ruta relativa al archivo.

La directiva ``ref`` se usa para hacer enlaces a partes específicas de una página.
Estos pueden ser encabezados, imágenes o secciones de código dentro de la página actual u otra.

Se necesita definir explícitamente una referencia justo antes del elemento deseado.
En el siguiente ejemplo, la referencia se define como ``_talker-listener`` una línea antes del título ``Prueba algunos ejemplos``.

.. code-block:: rst

   .. _talker-listener:

    Prueba algunos ejemplos
    -----------------

Ahora se puede crear el enlace desde cualquier página de la documentación a ese encabezado.

.. code-block:: rst

   :ref:` demo talker-listener <talker-listener>`

Este enlace llevará al lector a la página de destino con un enlace de anclaje HTML ``#talker-listener``.

Macros
~~~~~~

Las macros se pueden usar para simplificar la escritura de documentación dirigida a múltiples distribuciones.

Use una macro incluyendo el nombre de la macro entre llaves.
Por ejemplo, al generar los documentos para Rolling en la rama ``rolling``:


=====================  =========================  ==================================
Usando                 Resultado (para Rolling)   Ejemplo
=====================  =========================  ==================================
\{DISTRO\}             rolling                    ros-\{DISTRO\}-pkg
\{DISTRO_TITLE\}       Rolling                    ROS 2 \{DISTRO_TITLE\}
\{DISTRO_TITLE_FULL\}  Rolling Ridley             ROS 2 \{DISTRO_TITLE_FULL\}
\{REPOS_FILE_BRANCH\}  rolling                    git checkout \{REPOS_FILE_BRANCH\}
=====================  =========================  ==================================

El mismo archivo se puede usar en varias sucursales (es decir, para varias distribuciones) y el contenido generado será específico de la distribución.
