.. redirect-from::

    Quality-Guide
    Contributing/Quality-Guide

Guía de calidad: garantizar la calidad del código
=================================================

.. contents:: Tabla de Contenido
   :depth: 2
   :local:

Esta página brinda orientación sobre cómo mejorar la calidad del software de los paquetes ROS 2, centrándose en áreas más específicas que la sección Prácticas de calidad de la `Guía para desarrolladores <Developer-Guide>`.

Las siguientes secciones pretenden abordar los paquetes básicos, de aplicaciones y de ecosistemas de ROS 2 y las bibliotecas principales de clientes, C++ y Python.
Las soluciones presentadas están motivadas por consideraciones de diseño e implementación para mejorar atributos de calidad como "Confiabilidad", "Seguridad", "Mantenibilidad", "Determinismo", etc. que se relacionan con requisitos no funcionales.


Análisis de código estático como parte de la compilación del paquete ament
--------------------------------------------------------------------------

**Contexto**:


* Has desarrollado tu código de producción C++.
* Has creado un paquete ROS 2 con soporte de compilación con ``ament``.

**Problema**:


* El análisis de código estático a nivel de biblioteca no se ejecuta como parte del procedimiento de creación del paquete.
* El análisis de código estático a nivel de biblioteca debe ejecutarse manualmente.
* Riesgo de olvidarse de ejecutar el análisis de código estático a nivel de biblioteca antes de construir
   una nueva versión del paquete.

**Solución**:


* Usa las capacidades de integración de ``ament`` para ejecutar análisis de código estático como
   parte del procedimiento de construcción del paquete.

**Implementación**:


* Insertar en los paquetes el archivo ``CMakeLists.txt``.

.. code-block:: bash

   ...
   if(BUILD_TESTING)
     find_package(ament_lint_auto REQUIRED)
     ament_lint_auto_find_test_dependencies()
     ...
   endif()
   ...


* Inserte las dependencias de prueba ``ament_lint`` en el archivo ``package.xml`` de los paquetes.

.. code-block:: bash

   ...
   <package format="2">
     ...
     <test_depend>ament_lint_auto</test_depend>
     <test_depend>ament_lint_common</test_depend>
     ...
   </package>

**Ejemplos**:


* ``rclcpp``:

  * `rclcpp/rclcpp/CMakeLists.txt <https://github.com/ros2/rclcpp/blob/{REPOS_FILE_BRANCH}/rclcpp/CMakeLists.txt>`__
  * `rclcpp/rclcpp/package.xml <https://github.com/ros2/rclcpp/blob/{REPOS_FILE_BRANCH}/rclcpp/package.xml>`__

* ``rclcpp_lifecycle``:

  * `rclcpp/rclcpp_lifecycle/CMakeLists.txt <https://github.com/ros2/rclcpp/blob/{REPOS_FILE_BRANCH}/rclcpp_lifecycle/CMakeLists.txt>`__
  * `rclcpp/rclcpp_lifecycle/package.xml <https://github.com/ros2/rclcpp/blob/{REPOS_FILE_BRANCH}/rclcpp_lifecycle/package.xml>`__

**Contexto resultante**:


* Las herramientas de análisis de código estático soportadas por ``ament`` se ejecutan como parte de la construcción del paquete.
* Las herramientas de análisis de código estático que no son compatibles con ``ament`` deben ejecutarse por separado.

Análisis estático de seguridad de hilos a través de la anotación de código
--------------------------------------------------------------------------------

**Contexto:**


* Estás desarrollando/depurando tu código de producción C++ multihilo
* Accede a datos de múltiples hilos en código C++

**Problema:**


* Las carreras de datos y los interbloqueos pueden provocar errores críticos.

**Solución:**


* Utiliza el `Análisis de seguridad de hilos estático de Clang <https://clang.llvm.org/docs/ThreadSafetyAnalysis.html>`__ al anotar el código de subproceso

**Contexto para la implementación:**


Para habilitar el Análisis de Seguridad de hilos, se debe anotar el código para que el compilador sepa más sobre la semántica del código. Estas anotaciones son atributos específicos de Clang, p. ``__atributo__(capacidad()))``. En lugar de usar esos atributos directamente, ROS 2 proporciona macros de preprocesador que se borran cuando se usan otros compiladores.

Estas macros se pueden encontrar en `rcpputils/thread_safety_annotations.hpp <https://github.com/ros2/rcpputils/blob/{REPOS_FILE_BRANCH}/include/rcpputils/thread_safety_annotations.hpp>`__

La documentación del análisis de seguridad de hilos establece
   El análisis de seguridad de hilos se puede usar con cualquier biblioteca de hilos, pero requiere que la API de hilos se incluya en clases y métodos que tengan las anotaciones adecuadas.

Hemos decidido que queremos que los desarrolladores de ROS 2 puedan usar primitivas de subprocesamiento ``std::`` directamente para su desarrollo. No queremos proporcionar nuestros propios tipos envueltos como se sugiere anteriormente.

Hay tres bibliotecas estándar de C++ a tener en cuenta
* La biblioteca estándar GNU ``libstdc++``: predeterminada en Linux, explícitamente a través de la opción del compilador ``-stdlib=libstdc++``
* La biblioteca estándar LLVM ``libc++`` (también llamada ``libcxx``) - predeterminada en macOS, establecida explícitamente por la opción del compilador ``-stdlib=libc++``
* La biblioteca estándar de Windows C++: no es relevante para este caso de uso

``libcxx`` anota sus implementaciones ``std::mutex`` y ``std::lock_guard`` para el análisis de seguridad de hilos. Cuando se usa GNU ``libstdc++`` , esas anotaciones no están presentes, por lo que el análisis de seguridad de hilos no se puede usar en tipos ``std::`` no encapsulados.

*Por lo tanto, para usar Thread Safety Analysis directamente con tipos* ``std::`` *, debemos usar* ``libcxx``


**Implementación:**


Las sugerencias de migración de código aquí de ninguna manera están completas - al escribir (o anotar las existentes) código multihilo, se te recomienda a utilizar tantas anotaciones como sea lógico para tu caso de uso. Sin embargo, ¡este paso a paso es un gran lugar para comenzar!

* Habilitación de análisis para paquete/objetivo

   Cuando el compilador de C++ es Clang, habilite el indicador ``-Wthread-safety``. Ejemplo a continuación para proyectos basados en CMake

  .. code-block:: cmake

     if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
       add_compile_options(-Wthread-safety)   # for your whole package
       target_compile_options(${MY_TARGET} PUBLIC -Wthread-safety)  # for a single library or executable
     endif()

* Código de anotación

   * Paso 1 - Anotar miembros de datos

     * Encuentre en cualquier lugar que ``std::mutex`` se use para proteger algunos datos de miembros
     * Agregue la anotación ``RCPPUTILS_TSA_GUARDED_BY(mutex_name)`` a los datos que están protegidos por el mutex

    .. code-block:: cpp

      class Foo {
      public:
        void incr(int amount) {
          std::lock_guard<std::mutex> lock(mutex_);
          bar += amount;
        }

        void get() const {
          return bar;
        }

      private:
        mutable std::mutex mutex_;
        int bar RCPPUTILS_TSA_GUARDED_BY(mutex_) = 0;
      };

   * Paso 2: corregir advertencias

     * En el ejemplo anterior, ``Foo::get`` producirá una advertencia del compilador. Para solucionarlo, bloquear antes de volver a la bar

    .. code-block:: cpp

      void get() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return bar;
      }

   * Paso 3 - (Opcional pero recomendado) Refactorizar el código existente a un patrón privado-Mutex

     Un patrón recomendado en código C++ multihilo es mantener siempre su ``mutex`` como miembro ``privado:`` de la estructura de datos. Esto hace que la seguridad de los datos sea la preocupación de la estructura contenedora, descargando esa responsabilidad de los usuarios de la estructura y minimizando el área superficial del código afectado.

     Hacer que sus bloqueos sean privados puede requerir repensar las interfaces de sus datos. Este es un gran ejercicio - aquí hay algunas cosas a considerar

     * Es posible que desees proporcionar interfaces especializadas para realizar análisis que requieran una lógica de bloqueo compleja, p. contar miembros en un conjunto filtrado de una estructura de mapa protegida por mutex, en lugar de devolver la estructura subyacente a los consumidores
     * Considere la posibilidad de copiar para evitar el bloqueo, donde la cantidad de datos es pequeña. Esto puede permitir que otros hilos sigan accediendo a los datos compartidos, lo que potencialmente puede conducir a un mejor rendimiento general.

   * Paso 4 - (Opcional) Habilitar análisis de capacidad negativa

     https://clang.llvm.org/docs/ThreadSafetyAnalysis.html#negative-capabilities

     El análisis de capacidad negativa te permite especificar "este bloqueo no debe mantenerse al llamar a esta función". Puede revelar posibles casos de punto muerto que otras anotaciones no pueden.

     * Donde especificas ``-Wthread-safety``, agrega el indicador adicional ``-Wthread-safety-negative``
     * En cualquier función que adquiera un bloqueo, a el patrón ``RCPPUTILS_TSA_REQUIRES(!mutex)``



* Cómo ejecutar el análisis

   * La granja de compilación ROS CI ejecuta un trabajo nocturno con ``libcxx``, que hará surgir cualquier problema en la pila principal de ROS 2 al marcarse como "Inestable" cuando Thread Safety Analysis genere advertencias
   * Para ejecuciones locales, tienes las siguientes opciones, todas equivalentes

     * Utiliza el mixin colcon `clang-libcxx <https://github.com/colcon/colcon-mixin-repository/blob/master/clang-libcxx.mixin>`__

       * ``construcción colcon --mixin clang-libcxx``
       * Solo puedes usar esto si tiene `mixins configurados para su instalación de colcon <https://github.com/colcon/colcon-mixin-repository/blob/master/README.md>`__

     * Pasar el compilador a CMake

       * ``colcon build --cmake-args -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_CXX_FLAGS='-stdlib=libc++ -D_LIBCPP_ENABLE_THREAD_SAFETY_ANNOTATIONS' -DFORCE_BUILD_VENDOR_PKG=ON --no-warn-unused-cli``

     * Anulación del compilador del sistema

       * ``CC=clang CXX=clang++ colcon build --cmake-args -DCMAKE_CXX_FLAGS='-stdlib=libc++ -D_LIBCPP_ENABLE_THREAD_SAFETY_ANNOTATIONS' -DFORCE_BUILD_VENDOR_PKG=ON --no-warn-unused-cli``



**Contexto resultante:**


* Los interbloqueos potenciales y las condiciones de ejecución aparecerán en el momento de la compilación, al usar Clang y ``libcxx``


Análisis dinámico (ejecución de datos y puntos muertos)
-------------------------------------------------------

**Contexto:**


* Estas desarrollando/depurando tu código de producción C++ multiproceso.
* Usas pthreads o C++11 threading + llvm libc++ (en el caso de ThreadSanitizer).
* No utilizas enlaces estáticos Libc/libstdc++ (en el caso de ThreadSanitizer).
* No creas ejecutables que no sean independientes de la posición (en el caso de ThreadSanitizer).

**Problema:**


* Las carreras de datos y los interbloqueos pueden provocar errores críticos.
* Las carreras de datos y los interbloqueos no se pueden detectar mediante el análisis estático (motivo: limitación del análisis estático).
* Las carreras de datos y los interbloqueos no deben aparecer durante la depuración/prueba del desarrollo (razón: por lo general, no se ejercen todas las rutas de control posibles a través del código de producción).

**Solución:**


* Utilizar una herramienta de análisis dinámico que se centre en la búsqueda de carreras de datos y puntos muertos (aquí clang ThreadSanitizer).

**Implementación:**


* Compila y vincula el código de producción con clang usando la opción ``-fsanitize=thread`` (esto instrumenta el código de producción).
* En caso de que se ejecute un código de producción diferente durante el análisis, considere la compilación condicional, p. `ThreadSanitizers _has_feature(thread_sanitizer) <https://clang.llvm.org/docs/ThreadSanitizer.html#has-feature-thread-sanitizer>`__.
* En caso de que no se instrumente algún código, considere `ThreadSanitizers _/*attribute*/_((no_sanitize("thread"))) <https://clang.llvm.org/docs/ThreadSanitizer.html#attribute-no- higienizar-hilo>`__.
* En caso de que algunos archivos no se instrumenten, considera la exclusión a nivel de archivo o función `ThreadSanitizers blacklisting <https://clang.llvm.org/docs/ThreadSanitizer.html#ignorelist>`__, más específicamente: `ThreadSanitizers Sanitizer Special Case List <https://clang.llvm.org/docs/SanitizerSpecialCaseList.html>`__ o con `ThreadSanitizers no_sanitize("thread") <https://clang.llvm.org/docs/ThreadSanitizer.html#ignorelist>`__ y usa la opción ``--fsanitize-blacklist``.

**Contexto resultante:**


* Mayor probabilidad de encontrar carreras de datos y puntos muertos en el código de producción antes de implementarlo.
* El resultado del análisis puede carecer de fiabilidad, herramienta en fase beta (en el caso de ThreadSanitizer).
* Overhead por instrumentación de código de producción (mantenimiento de ramas separadas para código de producción instrumentado/no instrumentado, etc.).
* El código instrumentado necesita más memoria por subproceso (en el caso de ThreadSanitizer).
* El código instrumentado asigna mucho espacio de direcciones virtuales (en el caso de ThreadSanitizer).
