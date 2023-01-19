.. redirect-from::

    Contributing/Windows-Tips-and-Tricks

Consejos y trucos de Windows
============================

.. contents:: Tabla de Contenido
   :depth: 2
   :local:

ROS 2 es compatible con Windows 10 como plataforma de nivel 1, lo que significa que todo el código que se incluye en el núcleo de ROS 2 debe ser compatible con Windows.
Para aquellos que están acostumbrados al desarrollo tradicional en Linux u otros sistemas similares a Unix, desarrollar en Windows puede ser un desafío.
Este documento pretende exponer algunas de esas diferencias.

Longitud máxima de la ruta
--------------------------
De forma predeterminada, Windows tiene una `longitud de ruta máxima <https://docs.microsoft.com/en-us/windows/win32/fileio/maximum-file-path-limitation>`__ de 260 caracteres.
En términos prácticos, 4 de esos caracteres siempre se utilizan para la letra de la unidad, los dos puntos, la barra invertida inicial y el carácter NULL final.
Eso significa que solo hay 256 caracteres disponibles para la *suma* de todas las partes de la ruta.
Esto tiene dos consecuencias prácticas para ROS 2:

* Algunos de los nombres de rutas internas de ROS 2 son bastante largos. Debido a esto, siempre recomendamos usar un nombre  corto para la ruta del directorio raíz de ROS 2, como ``C:\dev``.
* Al compilar ROS 2 desde la fuente, el modo de compilación aislado predeterminado de colcon puede generar nombres de ruta muy largos. Para evitar estos nombres de ruta muy largos, use ``--merge-install`` cuando construya en Windows.

**Nota**: es posible cambiar Windows para tener longitudes de ruta máximas mucho más largas.
Ver `este artículo <https://docs.microsoft.com/en-us/windows/win32/fileio/maximum-file-path-limitation?tabs=cmd#enable-long-paths-in-windows-10-version-1607-and-later>`__ para más información.

.. _Windows_Symbol_Visibility:

Visibilidad del símbolo
-----------------------
El compilador de Microsoft Visual C++ (MSVC) expone símbolos de una biblioteca de vínculos dinámicos (DLL) solo si se exportan explícitamente.
Los compiladores clang y gcc tienen una opción para hacer lo mismo, pero está desactivada de forma predeterminada.
Como resultado, cuando una biblioteca compilada previamente en Linux se compila en Windows, es posible que otras bibliotecas no puedan resolver los símbolos externos.
A continuación se muestran ejemplos de mensajes de error comunes que pueden deberse a que los símbolos no están expuestos:

.. code-block:: console

   error C2448: '__attribute__': function-style initializer appears to be a function definition
   'visibility': identifier not found

.. code-block:: console

   CMake Error at C:/ws_ros2/install/random_numbers/share/random_numbers/cmake/ament_cmake_export_libraries-extras.cmake:48 (message):
      Package 'random_numbers' exports the library 'random_numbers' which
      couldn't be found

La visibilidad del símbolo también afecta la carga binaria.
Si encuentra que un nodo composable no se ejecuta o un Qt Visualizer no funciona, es posible que el proceso de alojamiento no pueda encontrar una exportación de símbolo esperada del binario.
Para diagnosticar esto en Windows, las herramientas de desarrollo de Windows incluyen un programa llamado Gflags para habilitar varias opciones.
Una de esas opciones se llama *Loader Snaps*, que le permite detectar errores de carga durante la depuración.
Visite la Documentación de Microsoft para obtener más información sobre `Gflags <https://docs.microsoft.com/en-us/windows-hardware/drivers/debugger/setting-and-clearing-image-file-flags>`__  y `Loaders snaps <https://docs.microsoft.com/en-us/windows-hardware/drivers/debugger/show-loader-snaps>`__.

Dos soluciones para exportar símbolos en Windows son los encabezados de control de visibilidad y la propiedad ``WINDOWS_EXPORT_ALL_SYMBOLS``.
Microsoft recomienda a los desarrolladores de ROS que utilicen encabezados de control de visibilidad para controlar la exportación de símbolos desde un binario.
Los encabezados de control de visibilidad brindan más control sobre la macro de exportación de símbolos y ofrecen otros beneficios, incluido un tamaño binario más pequeño y tiempos de enlace reducidos.

Encabezados de control de visibilidad
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
El propósito de los encabezados de control de visibilidad es definir una macro para cada biblioteca compartida que declare correctamente los símbolos como dllimport o dllexport.
Esto se decide en función de si la biblioteca se consume o se construye por sí misma.
La lógica de la macro también tiene en cuenta el compilador e incluye lógica para seleccionar la sintaxis adecuada.
La `documentación de visibilidad de GCC  <https://gcc.gnu.org/wiki/Visibility>`__ incluye instrucciones paso a paso para agregar visibilidad de símbolos explícitos a una biblioteca "produciendo el código de la más alta calidad con las mayores reducciones en tamaño binario, tiempos de carga y tiempos de enlace".
Se puede colocar un encabezado llamado ``visibility_control.h`` en la carpeta ``includes`` para cada biblioteca, como se muestra en el siguiente ejemplo.
El siguiente ejemplo muestra cómo se agregaría un encabezado de control de visibilidad para una biblioteca ``my_lib`` con una clase llamada ``example_class``.
Agregue un encabezado de visibilidad a la carpeta de inclusión de la biblioteca.
La  boiler plate logic (secciones de código que se repiten en múltiples lugares) se utiliza con el nombre de la biblioteca utilizada en la macro para que sea única en el proyecto.
En otra biblioteca, ``MY_LIB`` se reemplazaría con el nombre de la biblioteca.

.. code-block:: c++

   #ifndef MY_LIB__VISIBILITY_CONTROL_H_
   #define MY_LIB__VISIBILITY_CONTROL_H_
   #if defined _WIN32 || defined __CYGWIN__
   #ifdef __GNUC__
      #define MY_LIB_EXPORT __attribute__ ((dllexport))
      #define MY_LIB_IMPORT __attribute__ ((dllimport))
   #else
      #define MY_LIB_EXPORT __declspec(dllexport)
      #define MY_LIB_IMPORT __declspec(dllimport)
   #endif
   #ifdef MY_LIB_BUILDING_LIBRARY
      #define MY_LIB_PUBLIC MY_LIB_EXPORT
   #else
      #define MY_LIB_PUBLIC MY_LIB_IMPORT
   #endif
   #define MY_LIB_PUBLIC_TYPE MY_LIB_PUBLIC
   #define MY_LIB_LOCAL
   #else
    // Linux visibility settings
   #define MY_LIB_PUBLIC_TYPE
   #endif
   #endif  // MY_LIB__VISIBILITY_CONTROL_H_

Para ver un ejemplo completo de este encabezado, consulte `rviz_rendering <https://github.com/ros2/rviz/blob/ros2/rviz_rendering/include/rviz_rendering/visibility_control.hpp>`__.

Para usar la macro, agregue ``MY_LIB_PUBLIC`` antes de los símbolos que deben ser visibles para las bibliotecas externas. Por ejemplo:

.. code-block:: c++

   Class MY_LIB_PUBLIC example_class {}

   MY_LIB_PUBLIC void example_function (){}

La propiedad se puede implementar agregando lo siguiente al archivo

.. code-block:: cmake

  target_compile_definitions(${PROJECT_NAME}
    PRIVATE "MY_LIB_BUILDING_LIBRARY")


WINDOWS_EXPORT_ALL_SYMBOLS Target Property
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
CMake implementa la propiedad ``WINDOWS_EXPORT_ALL_SYMBOLS`` en Windows, lo que hace que los símbolos de función se exporten automáticamente.
Se pueden encontrar más detalles de cómo funciona en `WINDOWS_EXPORT_ALL_SYMBOLS CMake Documentation <https://cmake.org/cmake/help/latest/prop_tgt/WINDOWS_EXPORT_ALL_SYMBOLS.html>`__.
La propiedad se puede implementar agregando lo siguiente al archivo CMakeLists:

.. code-block:: cmake

    set_target_properties(${LIB_NAME} PROPIEDADES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

Si hay más de una biblioteca en un archivo CMakeLists, deberá llamar a ``set_target_properties`` en cada una de ellas por separado.

Tenga en cuenta que un binario en Windows solo puede exportar 65 536 símbolos.
Si un binario exporta más que eso, obtendrá un error y debe usar los encabezados de control de visibilidad.
Hay una excepción a este método en el caso de los símbolos de datos globales.
Por ejemplo, un miembro de datos estáticos globales como el siguiente.

.. code-block:: c++

   class Example_class
   {
   public:
   static const int Global_data_num;


En estos casos, dllimprort/dllexport debe aplicarse explícitamente.
Esto se puede hacer usando generate_export_header como se describe en el siguiente artículo: `Crear dlls en Windows sin declspec() usando la nueva característica de exportación de CMake <https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/>`__.

Finalmente, es importante que el archivo de encabezado que exporta los símbolos se incluya en al menos uno de los archivos ``.cpp`` en el paquete para que las macros se expandan y se coloquen en el binario resultante.
De lo contrario, los símbolos seguirán sin poderse llamar.


Debug builds
------------
Al compilar en modo de depuración en Windows, cambian varias cosas muy importantes.
La primera es que se agrega  automáticamente  ``_d`` al final del nombre de todas las DLL.
Entonces, si la biblioteca se llama ``libfoo.dll``, en el modo de depuración será ``libfoo_d.dll``.
El enlazador dinámico en Windows también sabe buscar bibliotecas de esa forma, por lo que no encontrará bibliotecas sin el prefijo ``_d``.
Además, Windows activa un conjunto completo de comprobaciones en tiempo de compilación y tiempo de ejecución en el modo de depuración que es mucho más estricto que las compilaciones de lanzamiento.
Por estas razones, es una buena idea ejecutar una compilación y prueba de depuración de Windows en muchas solicitudes de incorporación de cambios.

Barra inclinada frente a barra inclinada inversa
------------------------------------------------
En Windows, el separador de ruta predeterminado es una barra diagonal inversa (``\``), que difiere de la barra diagonal (``/``) utilizada en Linux y macOS.
La mayoría de las API de Windows pueden funcionar como un separador de ruta, pero esto no es universalmente cierto.
Por ejemplo, el shell ``cmd.exe`` solo puede completar con tabulación cuando se usa el carácter de barra invertida, no la barra diagonal.
Para lograr la máxima compatibilidad en Windows, siempre se debe usar una barra invertida como separador de ruta en Windows.

Parches de vendored packages
----------------------------
Al hacer copias locales de un paquete de terceros (vendoring) en ROS 2, a menudo es necesario aplicar un parche para corregir un error, agregar una característica, etc.
La forma típica de hacer esto es modificar la llamada ``ExternalProject_add`` para agregar un comando ``PATCH``, usando el ejecutable ``patch``.
Desafortunadamente, el ejecutable ``patch`` tal como lo entrega chocolatey requiere acceso de administrador para ejecutarse.
La solución es usar ``git apply-patch`` al aplicar parches a proyectos externos.

``git apply-patch`` tiene sus propios problemas, ya que solo funciona correctamente cuando se aplica a un repositorio de git.
Por esa razón, los proyectos externos siempre deben usar el método ``GIT`` para obtener el proyecto y luego usar ``PATCH_COMMAND`` para invocar ``git apply-patch``.

Un ejemplo de uso de todo lo anterior se ve asi:

.. code-block:: cmake

  ExternalProject_Add(mylibrary-${version}
    GIT_REPOSITORY https://github.com/lib/mylibrary.git
    GIT_TAG ${version}
    GIT_CONFIG advice.detachedHead=false
    # Suppress git update due to https://gitlab.kitware.com/cmake/cmake/-/issues/16419
    # See https://github.com/ament/uncrustify_vendor/pull/22 for details
    UPDATE_COMMAND ""
    TIMEOUT 600
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install
      ${extra_cmake_args}
      -Wno-dev
    PATCH_COMMAND
      ${CMAKE_COMMAND} -E chdir <SOURCE_DIR> git apply -p1 --ignore-space-change --whitespace=nowarn ${CMAKE_CURRENT_SOURCE_DIR}/install-patch.diff
  )

Tiempos más lentos en Windows  (lentitud en general)
----------------------------------------------------
El software que se ejecuta en Windows es, en general, mucho más lento que el que se ejecuta en Linux.
Esto se debe a una serie de factores, desde el intervalo de tiempo predeterminado (cada 20 ms, según la `documentación <https://docs.microsoft.com/en-us/windows/win32/procthread/multitasking>`__) , a la cantidad de procesos antivirus y antimalware en ejecución, a la cantidad de procesos en segundo plano en ejecución.
Debido a todo esto, las pruebas *nunca* deben esperar tiempos ajustados en Windows.
Todas las pruebas deben tener tiempos de espera generosos y solo esperar que los eventos sucedan eventualmente (esto también evitará que las pruebas sean inestables en Linux).

Shells
------
Hay dos shells principales de línea de comandos en Windows: el venerable ``cmd.exe`` y PowerShell.

``cmd.exe`` es el shell de comandos que más se parece al antiguo shell de DOS, aunque con capacidades muy mejoradas.
Está completamente basado en texto y sólo entiende archivos ``batch`` files de DOS/Windows.

PowerShell es el shell más nuevo basado en objetos que Microsoft recomienda para la mayoría de las aplicaciones nuevas.
Comprende archivos ``ps1`` para la configuración.

ROS 2 es compatible tanto con ``cmd.exe`` como con PowerShell, por lo que cualquier cambio (especialmente en cosas como ``ament`` o ``colcon``) debe probarse en ambos.
