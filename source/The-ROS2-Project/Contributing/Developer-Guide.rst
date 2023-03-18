.. redirect-from::

    Developer-Guide
    Contributing/Developer-Guide

Guía para desarrolladores de ROS 2
==================================

.. contents:: Tabla de contenido
   :depth: 2
   :local:

Esta página define las prácticas y políticas que empleamos al desarrollar ROS 2.

Principios generales
--------------------

Algunos principios son comunes a todo el desarrollo de ROS 2:


* **Propiedad compartida**:
  Todos los que trabajan en ROS 2 deben sentirse dueños de todas las partes del sistema.
  El autor original de un fragmento de código no tiene ningún permiso u obligación especial para controlar o mantener ese fragmento de código.
  Todos son libres de proponer cambios en cualquier lugar, manejar cualquier tipo de ticket y revisar cualquier pull request.
* **Estar dispuesto a trabajar en cualquier cosa**:
  Como corolario de la propiedad compartida, todos deben estar dispuestos a asumir cualquier tarea disponible y contribuir a cualquier aspecto del sistema.
* **Pedir ayuda**:
  Si tienes problemas con algo, solicita ayuda a tus compañeros desarrolladores, a través de tickets, comentarios o correo electrónico, según corresponda.

Prácticas de Calidad
--------------------

Los paquetes pueden categorizarse en diferentes niveles de calidad basado en las prácticas de desarrollo a las que se adhieran, según las directrices de `REP 2004: Package Quality Categories <https://www.ros.org/reps/rep-2004.html>`_ .
Las categorías se diferencian por sus políticas sobre versiones, pruebas, documentación y más.

Las siguientes secciones son las reglas de desarrollo específicas que seguimos para garantizar que los paquetes principales sean de la más alta calidad ('Nivel 1').
Recomendamos a todos los desarrolladores de ROS que se esfuercen por cumplir con las siguientes políticas para garantizar la calidad en todo el ecosistema de ROS.

.. _semver:

Versionado
^^^^^^^^^^

Usaremos las `directrices de control de versiones semánticas <https://semver.org/lang/es/>`__ (``semver``) para el control de versiones.

También nos adherimos a algunas reglas específicas de ROS adicionales encima de lo definido en ``semver``:

* Los incrementos de versiones Mayores (es decir, cambios incompatibles) no deben realizarse dentro de una distribución de ROS publicada.

  * Los parches (con preservación de la interfaz) y los incrementos de versión Menores (sin interrupciones) no rompen la compatibilidad, por lo que este tipo de cambios *están* permitidos dentro de una versión.

  * Los lanzamientos Mayores de ROS son el mejor momento para lanzar cambios incompatibles (breaking change).
    Si un paquete principal necesita varios cambios incompatibles, deben fusionarse en su rama de integración (por ejemplo, rolling) para permitir detectar problemas en CI rápidamente, pero deben publicarse juntos para reducir la cantidad de versiones Major para los usuarios de ROS.

  * Aunque los incrementos importantes requieren una nueva distribución, una nueva distribución no necesariamente requiere un aumento importante (si el desarrollo y el lanzamiento pueden ocurrir sin interrumpir la API).

* Para el código compilado, la ABI (Interfaz binaria de aplicaciones) se considera parte de la interfaz pública.
   Cualquier cambio que requiera volver a compilar el código dependiente se considera cambio incompatible.

  * Los cambios incompatibles de ABI *pueden* realizarse en una actualización de versión Menor *antes* de un lanzamiento de distribución  (agregándola a la distribución rolling).

* Imponemos la estabilidad de la API para los paquetes principales en Dashing y Eloquent aunque sus componentes principales de la versión sean ``0``, a pesar de la `especificación de SemVer <https://semver.org/lang/es/#spec-item-4>`_ con respecto al desarrollo inicial.

  * Posteriormente, los paquetes deben esforzarse por alcanzar un estado maduro y aumentar a la versión ``1.0.0`` para que coincidan con las especificaciones de ``semver``.

Advertencias
~~~~~~~~~~~~

Estas reglas son de *mejor esfuerzo*.
En casos extremos e improbables, puede ser necesario romper la API dentro de una versión/distribución principal.
Para un caso de cambio no planificado que rompa compatibilidad se evaluará si es considerado un incremento Mayor o Menor.

Por ejemplo, considere una situación que involucre el lanzamiento de X-turtle, correspondiente a la versión Mayor ``1.0.0``, y Y-turtle lanzado, correspondiente a la versión Mayor ``2.0.0``.

Si se identifica que un arreglo con ruptura de la API es absolutamente necesaria en X-turtle, saltar a ``2.0.0`` obviamente no es una opción porque ``2.0.0`` ya existe.

Las soluciones para manejar la versión de X-turtle en tal caso, ambas no ideales, son:

1. Actualizar X-turtle con una versión Menor: no es ideal porque viola el principio de SemVer de que los cambios incompatibles deben incrementar la versión Mayor.

2. Actualizar la versión Mayor de X-turtle más allá de Y-turtle (a ``3.0.0``): no es ideal porque la versión de la distribución anterior se volvería más alta que la versión ya disponible de una distribución más nueva, lo que invalidaría/rompería código condicional específico de la versión.

El desarrollador tendrá que decidir qué solución usar o, lo que es más importante, qué principio está dispuesto a romper.
No podemos sugerir uno u otro, pero en cualquier caso sí requerimos que se tomen medidas explícitas para comunicar la interrupción y su explicación a los usuarios de forma manual (más allá del incremento de versión).

Si no hubiera Y-turtle, aunque técnicamente la solución sería solo un parche, X-turtle tendría que pasar a ``2.0.0``.
Este caso se adhiere a SemVer, pero rompe con nuestra propia regla de que no se deben introducir incrementos importantes en una distribución publicada.

Es por eso que consideramos las reglas de control de versiones *mejor esfuerzo*.
Tan improbable como el ejemplo que se muestra arriba, es importante definir con precisión nuestro sistema de control de versiones.

Declaración de API pública
~~~~~~~~~~~~~~~~~~~~~~~~~~

Según ``semver``, cada paquete debe declarar claramente una API pública.
Usaremos la sección "Declaración de API pública" de la declaración de calidad de un paquete para declarar qué símbolos forman parte de la API pública.

Para la mayoría de los paquetes C y C++, la declaración es cualquier encabezado que instale.
Sin embargo, es aceptable definir un conjunto de símbolos que se consideran privados.
Evitar los símbolos privados en los encabezados puede ayudar con la estabilidad de ABI, pero no es obligatorio.

Para otros lenguajes como Python, se debe definir explícitamente una API pública, de modo que quede claro en qué símbolos se puede confiar con respecto a las pautas de control de versiones.
La API pública también se puede ampliar para crear artefactos como variables de configuración, archivos de configuración de CMake, etc., así como ejecutables y opciones y resultados de la línea de comandos.
Todos los elementos de la API pública deben indicarse claramente en la documentación del paquete.
Si algo que estas utilizando no aparece explícitamente como parte de la API pública en la documentación del paquete, entonces no puedes asumir en que no cambie entre versiones Menores o parches.

Estrategia de Obsolescencia
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Siempre que sea posible, también utilizaremos la estrategia de migración y obsolescencia tick-tock para los incrementos de versiones Mayores.
Las nuevas obsolescencias vendrán en una nueva versión de distribución, acompañadas de advertencias del compilador que expresan que la funcionalidad está obsoleta.
En la próxima versión, la funcionalidad se eliminará por completo (sin advertencias).

Ejemplo de la función ``foo`` obsoleta y reemplazada por la función ``bar``:

=========  ========================================================
  Versión   API
=========  ========================================================
X-turtle   void foo();
Y-turtle   [[deprecated("use bar()")]] void foo(); <br> void bar();
Z-turtle   void bar();
=========  ========================================================

No debemos agregar obsolescencias después de lanzar una distribución.
Sin embargo, las obsolescencias no requieren necesariamente un cambio de versión Mayor.
Se puede introducir una obsolescencia en un aumento de versión Menor si el aumento ocurre antes de que se lance la distribución (similar a los cambios incompatibles de ABI).

Por ejemplo, si X-turtle comienza a desarrollarse como ``2.0.0``, se puede agregar una obsolescencia en ``2.1.0`` antes de lanzar X-turtle.

Intentaremos mantener la compatibilidad entre distribuciones tanto como sea posible.
Sin embargo, al igual que las advertencias asociadas con SemVer, el tic-tac o incluso la obsolescencia en general puede ser imposible de cumplir por completo en ciertos casos.

Proceso de control de cambios
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Todos los cambios deben pasar por un pull request.

* Haremos cumplir el `Certificado de origen del desarrollador (DCO) <https://developercertificate.org/>`_ en los pull request a los repositorios de ROSCore.

  * Requiere que todos los mensajes de confirmación contengan la línea ``Firmado por`` con una dirección de correo electrónico que coincida con el autor de la confirmación.

  * Puede pasar ``-s`` / ``--signoff`` a la invocación ``git commit`` o escribir el mensaje esperado manualmente (por ejemplo, ``Firmado por: Su nombre Desarrollador <su.nombre @ejemplo.com>``).

  * DCO *no* se requiere para los pull request que solo abordan la eliminación de espacios en blanco, la corrección de errores tipográficos y otros `cambios triviales <http://cr.openjdk.java.net/~jrose/draft/trivial-fixes.html>`_ .

* Ejecute siempre trabajos de CI para todas las `plataformas de nivel 1 <https://www.ros.org/reps/rep-2000.html#support-tiers>`_ para cada pull request e incluya enlaces a trabajos en el pull request.
   (Si no tiene acceso a los trabajos de Jenkins, alguien activará los trabajos por usted).

* Se requiere un mínimo de 1 aprobación de un compañero desarrollador que no haya creado el pull request para considerarlo aprobado.
  Se requiere aprobación antes de realizar la fusión.

  * Los paquetes pueden optar por aumentar este número.

* Cualquier cambio requerido en la documentación (documentación de la API, documentación de funciones, notas de la versión, etc.) debe proponerse antes de fusionar los cambios relacionados.

Directrices para aplicar los PR a versiones anteriores
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Al cambiar una versión anterior (backporting) de ROS:

* Asegúrate de que las funciones o las correcciones se acepten y fusionen en la rama rolling antes de abrir una PR para aplicar los cambios a versiones anteriores.
* Al realizar una adaptación a versiones anteriores, también considere la posibilidad de realizar una adaptación a cualquier otra :doc:`versión aún compatible <../../Releases>`, incluso versiones que no sean LTS.
* Si está realizando backport de un solo PR en su totalidad, titule el PR de retroceso "[Distro] <nombre del PR original>".
  Si se hace un backport un de subconjunto de cambios provenientes de uno o varios PR, el título debe ser "[Distro] <descripción de los cambios>".
* Vincula a todos los PR cuyos cambios estés realizando su backport desde la descripción del backport PR.
  En un backport de Foxy a Dashing no es necesario enlazar el backport a Eloquent en el mismo cambio.

Documentación
^^^^^^^^^^^^^

Todos los paquetes deben tener estos elementos de documentación presentes en su README o vinculados desde su README:

* Descripción y propósito
* Definición y descripción de la API pública
* Ejemplos
* Cómo compilar e instalar (debe hacer referencia a herramientas/flujos de trabajo externos)
* Cómo construir y ejecutar pruebas
* Cómo construir la documentación
* Cómo desarrollar (útil para describir cosas como ``python setup.py Develop``)
* Declaraciones de licencia y derechos de autor

Cada archivo fuente debe tener una licencia y una declaración de derechos de autor, verificada con un linter automático.

Cada paquete debe tener un archivo de LICENCIA, generalmente la licencia de Apache 2.0, a menos que el paquete tenga una licencia permisiva existente (por ejemplo, rviz usa BSD de tres cláusulas).

Cada paquete debe describirse a sí mismo y su propósito suponiendo, en la medida de lo posible, que el lector se ha topado con él sin conocimiento previo de ROS u otros proyectos relacionados.

Cada paquete debe definir y describir su API pública para que haya una expectativa razonable para los usuarios sobre lo que cubre la política de versiones semánticas.
Incluso en C y C++, donde la API pública se puede aplicar mediante la verificación de API y ABI, es una buena oportunidad para describir el diseño del código y la función de cada parte del código.

Debe ser fácil tomar cualquier paquete y, a partir de la documentación de ese paquete, comprender cómo construir, ejecutar, construir y ejecutar pruebas, y como construir la documentación.
Obviamente, debemos evitar repetirnos para flujos de trabajo comunes, como crear un paquete en un espacio de trabajo, pero los flujos de trabajo básicos deben describirse o referenciarse.

Finalmente, debe incluir cualquier documentación para desarrolladores.
Esto podría incluir flujos de trabajo para probar el código usando algo como ``python setup.py Develop``, o podría significar describir cómo hacer uso de los puntos de extensión proporcionados por su paquete.

Ejemplos:

* capacidades: https://docs.ros.org/hydro/api/capabilities/html/

   * Este da un ejemplo de documentos que describen la API pública

* catkin_tools: https://catkin-tools.readthedocs.org/en/latest/development/extending_the_catkin_command.html

   * Este es un ejemplo de descripción de un punto de extensión para un paquete

*(Los documentos API aún no se generan automáticamente)*

Pruebas
^^^^^^^

Todos los paquetes deben tener algún nivel de :ref:`sistema, integración y/o pruebas unitarias.<TestingMain>`

**Las pruebas unitarias** siempre deben estar en el paquete que se está probando y deben usar herramientas como ``Mock`` para probar y verificar partes especificas de la base de código en escenarios construidos.
Las pruebas unitarias no deben incluir dependencias de prueba que no sean herramientas de prueba, p. gtest, nosetest, pytest, simulacro, etc...

**Las pruebas de integración** pueden probar interacciones entre partes del código o entre partes del código y el sistema.
A menudo prueban las interfaces de software de la forma en que esperamos que el usuario las use.
Al igual que las pruebas unitarias, las pruebas de integración deben estar en el paquete que se está probando y no deben incluir dependencias de prueba que no sean de herramientas a menos que sea absolutamente necesario, es decir, todas las dependencias que no sean de herramientas solo deben permitirse bajo un escrutinio extremo, por lo que deben evitarse si es posible.

**Las pruebas del sistema** están diseñadas para probar situaciones de un extremo a otro entre paquetes y deben estar en sus propios paquetes para evitar la sobrecarga o el acoplamiento de paquetes y para evitar dependencias circulares.

En general, se debe evitar minimizar las dependencias de prueba de paquetes cruzados o externos para evitar dependencias circulares y paquetes de prueba estrechamente acoplados.

Todos los paquetes deben tener algunas pruebas unitarias y posiblemente pruebas de integración, pero el grado en que deben tenerlas se basa en la categoría de calidad del paquete.
Las siguientes subsecciones se aplican a los paquetes de 'Nivel 1':

Cobertura de código
~~~~~~~~~~~~~~~~~~~

Proporcionaremos cobertura de línea y lograremos una cobertura de línea superior al 95%.
Si se justifica un objetivo de porcentaje más bajo, debe documentarse de forma explicita.
Podemos proporcionar cobertura de rama o excluir código de la cobertura (código de prueba, código de depuración, etc.).
Requerimos que la cobertura aumente o permanezca igual antes de fusionar un cambio, pero puede ser aceptable hacer un cambio que reduzca la cobertura del código con la justificación adecuada (p. ej., eliminar el código que se cubrió anteriormente puede hacer que el porcentaje baje).

Rendimiento
~~~~~~~~~~~

Recomendamos fuertemente las pruebas de rendimiento, pero reconocemos que no tienen sentido para algunos paquetes.
Si hay pruebas de rendimiento, elegiremos verificar cada cambio o antes de cada lanzamiento o ambos.
También necesitaremos una justificación para fusionar un cambio o hacer una versión que reduzca el rendimiento.

Linters y análisis estático
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Usaremos el :doc:`estilo de código ROS <Code-Style-Language-Versions>` y lo aplicaremos con linters de `ament_lint_common <https://github.com/ament/ament_lint/tree/{REPOS_FILE_BRANCH}/ament_lint_common/doc /index.rst>`_.
Se deben usar todos los análisis estáticos/linters que forman parte de ``ament_lint_common``.

La documentación de `ament_lint_auto <https://github.com/ament/ament_lint/blob/{REPOS_FILE_BRANCH}/ament_lint_auto/doc/index.rst>`_ proporciona información sobre cómo ejecutar ``ament_lint_common``.

Prácticas Generales
-------------------

Algunas prácticas son comunes a todo el desarrollo de ROS 2.

Estas prácticas no afectan el nivel de calidad del paquete como se describe en `REP 2004 <https://www.ros.org/reps/rep-2004.html>`_, pero siguen siendo muy recomendables para el proceso de desarrollo.

Issues
^^^^^^

Al presentar un issue, Asegúrate de:

- Incluir suficiente información para que otra persona entienda el problema.
  En ROS 2, los siguientes puntos pueden utilizarse para reducir el motivo de un problema.
  Realizar pruebas con tantas alternativas en cada categoría como sea posible será de especial ayuda.

  - **El sistema operativo y la versión.**
    Razonamiento: ROS 2 es compatible con múltiples plataformas y algunos errores son específicos de versiones particulares de sistemas operativos/compiladores.
  - **El método de instalación.**
    Razonamiento: algunos problemas solo se manifiestan si ROS 2 se ha instalado desde "archivos fat" o desde Debians.
    Esto puede ayudarnos a determinar si el problema está relacionado con el proceso de empaquetado.
  - **La versión específica de ROS 2.**
    Razonamiento: algunos errores pueden estar presentes en una versión particular de ROS 2 y luego se corrigieron.
    Es importante saber si su instalación incluye estas correcciones.
  - **La implementación de DDS/RMW que se está utilizando** (consulte `esta página <../../Concepts/About-Different-Middleware-Vendors>` para saber cómo determinar cuál).
    Razonamiento: los problemas de comunicación pueden ser específicos del middleware de ROS subyacente que se utiliza.
  - **La biblioteca cliente ROS 2 en uso.**
    Razonamiento: esto nos ayuda a reducir la capa en la pila en la que podría estar el problema.

- Incluir una lista de pasos para reproducir el problema.
- En caso de un error, considera proporcionar un `ejemplo breve, autónomo, correcto (compilable) <http://sscce.org/>`__.
   Es mucho más probable que los problemas se resuelvan si otros pueden reproducirlos fácilmente.

- Menciona los pasos de solución de problemas que ya se han intentado, incluidos:

   - Actualización a la última versión del código, que puede incluir correcciones de errores que aún no se han publicado.
     Consulta `esta sección <building-from-source>` y siga las instrucciones para obtener las ramas "rolling".
   - Probar con una implementación diferente de RMW.
     Consulte `esta página <../../How-To-Guides/Working-with-multiple-RMW-implementations>` para saber cómo hacerlo.

Ramas
^^^^^

.. note::
     Estas son solo pautas.
     Depende del mantenedor del paquete elegir los nombres de las ramas que coincidan con su propio flujo de trabajo.

En el repositorio código fuente es una buena práctica tener **ramas separadas** para cada una de las distribuciones de ROS a la que se dirige el paquete.
Estas ramas suelen tener el nombre de la distribución a la que se dirigen.
Por ejemplo, una rama ``humble`` para el desarrollo dirigida específicamente a la distribución Humble.

Desde estas ramas también se realizan lanzamiento, enfocándose en la distribución correspondiente.
El desarrollo dirigido a una distribución de ROS específica puede ocurrir en la rama apropiada.
Por ejemplo: las confirmaciones de desarrollo dirigidas a ``foxy`` se realizan en la rama ``foxy``, y los lanzamientos de paquetes para ``foxy`` se realizan desde esa misma rama.

.. note::
    Esto requiere que los mantenedores del paquete realicen backports o forwardports según corresponda para mantener todas las ramas actualizadas con las características.
    Los mantenedores también deben realizar el mantenimiento general (corrección de errores, etc.) en todas las ramas desde las que aún se realizan lanzamientos de paquetes.

    Por ejemplo, si una función se fusiona con la rama específica de Rolling (por ejemplo, ``rolling`` o ``main``), y esa función también es apropiada
    a la distribución de Humble (no rompe la API, etc.), entonces es una buena práctica transferir la función a la rama específica de Humble.

    Los mantenedores pueden hacer lanzamientos para esas distribuciones más antiguas si hay nuevas funciones o correcciones de errores disponibles.

**¿Qué pasa con** ``main`` **y** ``rolling`` **?**

``main`` generalmente apunta a :doc:`Rolling <../../Releases/Release-Rolling-Ridley>` (y por lo tanto, la próxima distribución de ROS inédita), aunque los mantenedores pueden decidir desarrollar y lanzar desde una rama ``rolling`` en su lugar.

Pull requests
^^^^^^^^^^^^^

* Un pull request solo debe centrarse en un cambio.
   Los cambios separados deben ir en pull request separados.
   Consulte `Guía de GitHub para escribir el pull request perfecta <https://github.com/blog/1943-how-to-write-the-perfect-pull-request>`__.

* Un parche debe tener un tamaño mínimo y evitar cualquier tipo de cambios innecesarios.

* Una pull request debe contener una cantidad mínima de commits significativos.

   * Puedes crear nuevos commits mientras se revisa el pull request.

* Antes de fusionar un pull request, todos los cambios deben agruparse (squash) en una pequeña cantidad de commits semánticos para mantener el historial claro.

   * Pero evita realizar un commit squash mientras se revisa el pull request.
     Es posible que tus revisores no se den cuenta de que realizaste el cambio, lo que podría generar confusión.
     Además, de todos modos vas a agrupar antes de fusionar; no hay ningún beneficio en hacerlo antes.

* Cualquier desarrollador puede revisar y aprobar una pull request (consulta `Principios generales`_).

* Cuando comiences a revisar un pull request, comenta el pull request para que otros desarrolladores sepan que lo estás revisando.

* La revisión de pull request no es de solo lectura, ya que el revisor hace comentarios y luego espera a que el autor los aborde.
   Como revisor, no dudes en realizar mejoras menores (errores tipográficos, problemas de estilo, etc.) en el lugar.
   Como autor de un pull request, si está trabajando en una fork, marque la casilla para `permitir ediciones de colaboradores upstream <https://github.com/blog/2247-improveing-collaboration-with-forks>`__ ayudará con lo antes mencionado.
   Como revisor, siéntete libre de realizar mejoras más sustanciales, pero considere colocarlas en una rama separada (mencione la nueva rama en un comentario o abra otra pull request de la nueva rama a la rama original).

* Cualquier desarrollador (el autor, el revisor u otra persona) puede fusionar cualquier pull request aprobado.

Versionado de biblioteca
^^^^^^^^^^^^^^^^^^^^^^^^

Versionaremos todas las bibliotecas dentro de un paquete juntas.
Esto significa que las bibliotecas heredan su versión del paquete.
Esto evita que las versiones de la biblioteca y del paquete diverjan y comparte el razonamiento con la política de lanzar paquetes que comparten un repositorio juntos.
Si necesita que las bibliotecas tengan diferentes versiones, considere dividirlas en diferentes paquetes.

Proceso de desarrollo
^^^^^^^^^^^^^^^^^^^^^

* La rama predeterminada (en la mayoría de los casos, la rama rolling) siempre debe compilarse, pasar todas las pruebas y compilarse sin advertencias.
   Si en algún momento hay una regresión, la máxima prioridad es restaurar al menos el estado anterior.
* Compila siempre con las pruebas habilitadas.
* Ejecuta siempre las pruebas localmente después de los cambios y antes de proponerlos en una pull request.
   Además de usar pruebas automatizadas, también ejecuta la ruta del código modificado manualmente para asegurarse de que el parche funcione según lo previsto.
* Siempre ejecuta trabajos de CI para todas las plataformas para cada pull request e incluye enlaces a los trabajos en el pull request.

Para obtener más detalles sobre el flujo de trabajo de desarrollo de software recomendado, consulta la sección `Ciclo de vida de desarrollo de software`_.

Cambios en la API de RMW
^^^^^^^^^^^^^^^^^^^^^^^^

Al actualizar `RMW API <https://github.com/ros2/rmw>`__, es necesario que también se actualicen las implementaciones de RMW para las bibliotecas de middleware de nivel 1.
Por ejemplo, una nueva función ``rmw_foo()`` introducida en la API de RMW debe implementarse en los siguientes paquetes (a partir de ROS Galactic):

* `rmw_connextdds <https://github.com/ros2/rmw_connextdds>`__
* `rmw_cyclonedds <https://github.com/ros2/rmw_cyclonedds>`__
* `rmw_fastrtps <https://github.com/ros2/rmw_fastrtps>`__

Las actualizaciones para bibliotecas de middleware que no sean de nivel 1 también deben considerarse si es factible (por ejemplo, dependiendo del tamaño del cambio).
Consulte `REP-2000 <https://www.ros.org/reps/rep-2000.html>`__ para ver la lista de bibliotecas de middleware y sus niveles.

Seguimiento de tareas
^^^^^^^^^^^^^^^^^^^^^

Para ayudar a organizar el trabajo en ROS 2, el equipo central de desarrollo de ROS 2 utiliza tableros de proyecto `GitHub de estilo kanban <https://github.com/orgs/ros2/projects>`_.

Sin embargo, no todas las propuestas y pull request se rastrean en los tableros de proyecto.
Un tablero generalmente representa un lanzamiento próximo o un proyecto específico.
Los tickets se pueden buscar por repositorio navegando en las páginas de issues individuales de los `repositorios de ROS 2' <https://github.com/ros2>`_.

Los nombres y propósitos de las columnas en cualquier tablero de proyecto de ROS 2 varían, pero normalmente siguen la misma estructura general:

* **Hacer**:
   Problemas que son relevantes para el proyecto, listos para ser asignados
* **En progreso**:
   pull request activos en los que se está trabajando actualmente
* **En revisión**:
   pull request donde el trabajo está completo y listo para revisión, y para aquellos que actualmente se encuentran en revisión activa
* **Hecho**:
   Los pull request y los problemas relacionados se fusionan/cierran (con fines informativos)

Para solicitar permiso para realizar cambios, simplemente comenta los issues que te interesan.
Dependiendo de la complejidad, puede ser útil describir cómo piensas abordarlo.
Actualizaremos el estado (si no tienes el permiso) y podrás comenzar a trabajar en un pull request.
Si contribuyes regularmente, es probable que simplemente te concedamos permiso para que puedas administrar por ti mismo las etiquetas, etc. .

Convenciones de programación
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Programación defensiva: asegúrate de que los supuestos se lleven a cabo lo antes posible.
   P.ej. verifica cada código de retorno y asegúrate de lanzar al menos una excepción hasta que el caso se maneje con más gracia.
* Todos los mensajes de error deben dirigirse a ``stderr``.
* Declarar variables en el ámbito más reducido posible.
* Mantener grupos de elementos (dependencias, importaciones, inclusiones, etc.) ordenados alfabéticamente.

específico de C++
~~~~~~~~~~~~~~~~~

* Evita el uso de transmisión directa (``<<``) a ``stdout`` / ``stderr`` para evitar la intercalación entre varios subprocesos.
* Evita usar referencias para ``std::shared_ptr`` ya que eso subvierte el conteo de referencias.
   Si la instancia original queda fuera del alcance y se está utilizando la referencia, accede a la memoria liberada.

Diseño del sistema de archivos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

El diseño del sistema de archivos de los paquetes y repositorios debe seguir las mismas convenciones para proporcionar una experiencia consistente para los usuarios que navegan nuestro código fuente.

Diseño del paquete
~~~~~~~~~~~~~~~~~~

* ``src``: contiene todo el código C y C++

  * También contiene encabezados C/C++ que no están instalados

* ``include``: contiene todos los encabezados C y C++ que están instalados

  * ``<nombre del paquete>``: para todos los encabezados instalados de C y C++, deben tener un espacio de carpeta con el nombre del paquete

* ``<package_name>``: contiene todo el código de Python
* ``test``: contiene todas las pruebas automatizadas y datos de prueba
* ``config``: contiene archivos de configuración, p. Archivos de parámetros YAML y archivos de configuración RViz
* ``doc``: contiene toda la documentación
* ``launch``: contiene todos los archivos de lanzamiento
* ``package.xml``: como se define en `REP-0140 <https://www.ros.org/reps/rep-0140.html>`_ (puede actualizarse para la creación de prototipos)
* ``CMakeLists.txt``: solo paquetes ROS que usan CMake
* ``setup.py``: solo paquetes ROS que usan solo código Python
* ``README``: se puede representar en GitHub como una página de destino para el proyecto

   * Esto puede ser tan breve o detallado como sea conveniente, pero al menos debe vincularse a la documentación del proyecto
   * Considere colocar una etiqueta de cobertura de código o CI en este README
   * También puede ser ``.rst`` o cualquier otra cosa compatible con GitHub

* ``CONTRIBUTING``: describe las pautas de contribución

   * Esto podría incluir implicaciones de licencia, p. cuando se utiliza la licencia Apache 2.

* ``LICENSE``: una copia de la licencia o licencias para este paquete
* ``CHANGELOG.rst``: `REP-0132 <https://www.ros.org/reps/rep-0132.html>`_ registro de cambios compatible

Diseño del repositorio
~~~~~~~~~~~~~~~~~~~~~~

Cada paquete debe estar en una subcarpeta que tenga el mismo nombre que el paquete.
Si un repositorio contiene un solo paquete, opcionalmente puede estar en la raíz del repositorio.

Flujo de trabajo del desarrollador
----------------------------------

Realizamos un seguimiento de los tickets abiertos y los pull request activos relacionados con los próximos lanzamientos y proyectos más grandes mediante `tableros de proyectos de GitHub <https://github.com/orgs/ros2/projects>`_.

El flujo de trabajo habitual es:

* Discutir el diseño (ticket de GitHub en el repositorio apropiado y un diseño PR a https://github.com/ros2/design si es necesario)
* Escribir la implementación en una rama feature en un fork

   * Consulta la `guía para desarrolladores <Developer-Guide>` para conocer las pautas y las mejores prácticas

* Escribir pruebas
* Habilitar y ejecutar linters
* Ejecutar pruebas localmente usando ``colcon test`` (consulta el :doc:`tutorial de colcon <../../Tutorials/Beginner-Client-Libraries/Colcon-Tutorial>`)
* Una vez que todo se compila localmente sin advertencias y todas las pruebas pasan, ejecute CI en su rama de características:

   * Ir a ci.ros2.org
   * Iniciar sesión (esquina superior derecha)
   * Haga clic en el trabajo ``ci_launcher``
   * Haga clic en "Construir con parámetros" (columna izquierda)
   * En el primer cuadro "CI_BRANCH_TO_TEST" ingresa el nombre de su sucursal de características
   * Presiona el botón ``construir``

   (si no es un confirmador de ROS 2, no tiene acceso a la granja de CI. En ese caso, haga ping al revisor de su PR para que ejecute el CI por ti)

* Si tu caso de uso requiere una cobertura de código en ejecución:

   * Ir a ci.ros2.org
   * Iniciar sesión (esquina superior derecha)
   * Haga clic en el trabajo ``ci_linux_coverage``
   * Haga clic en "Construir con parámetros" (columna izquierda)
   * Asegúrate de dejar "CI_BUILD_ARGS" y "CI_TEST_ARGS" con los valores predeterminados
   * Presiona el botón ``construir``
   * Al final del documento hay instrucciones sobre cómo :ref:`interpretar el resultado del informe <read-coverage-report>` y :ref:`calcular la tasa de cobertura <calculate-coverage-rate>`

* Si el trabajo de CI se generó sin advertencias, errores y fallas de prueba, publica los enlaces de sus trabajos en tu PR o ticket de alto nivel agregando todos tus PR (consulte el ejemplo `aquí <https://github.com/ros2/rcl/pull/106#issuecomment-271119200>`__)

   * Tenga en cuenta que el markdown para estas insignias está en la salida de la consola del trabajo ``ci_launcher``

* Cuando el PR ha sido aprobado:

   * la persona que envió el PR lo fusiona usando la opción "Squash and Merge" para que mantengamos un historial limpio

     * Si las confirmaciones merecen mantenerse separadas: aplaste todas las quisquillosas/linters/errores tipográficos y fusione el conjunto restante

       * Nota: cada PR debe apuntar a una función específica, por lo que Squash y Merge deberían tener sentido el 99% del tiempo

* Eliminar la rama una vez fusionada

Prácticas de desarrollo arquitectónico
--------------------------------------

Esta sección describe el ciclo de vida ideal que debe emplearse al realizar grandes cambios en la arquitectura de ROS 2.

Ciclo de vida de desarrollo de software
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Esta sección describe paso a paso cómo planificar, diseñar e implementar una nueva función:

1. Creación de tareas
2. Creando el Documento de Diseño
3. Revisión del diseño
4. Implementación
5. Revisión de código

Creación de tareas
~~~~~~~~~~~~~~~~~~

Las tareas que requieren cambios en partes críticas de ROS 2 deben tener revisiones de diseño durante las primeras etapas del ciclo de lanzamiento.
Si se está realizando una revisión del diseño en las etapas posteriores, los cambios serán parte de una versión futura.

* Se debe crear un issue en el repositorio `ros2 apropiado <https://github.com/ros2/>`__, que describa claramente la tarea en la que se está trabajando.

   * Debe tener un criterio de éxito claro y resaltar las mejoras concretas que se esperan de él.
   * Si la característica tiene como objetivo una versión de ROS, asegúrate de que se realice un seguimiento en el ticket de la versión de ROS (`example <https://github.com/ros2/ros2/issues/607>`__).

Redacción del documento de diseño.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Los documentos de diseño nunca deben incluir información confidencial.
El hecho de que se requiera o no un documento de diseño para su cambio depende de qué tan grande sea la tarea.

1. Estás haciendo un pequeño cambio o corrigiendo un error:

   * No se requiere un documento de diseño, pero se debe abrir un issue en el repositorio adecuado para realizar un seguimiento del trabajo y evitar la duplicación de esfuerzos.

2. Estás implementando una nueva función o te gustaría contribuir a la infraestructura propiedad de OSRF (como Jenkins CI):

   * El documento de diseño es obligatorio y debe contribuirse a `ros2/design <https://github.com/ros2/design/>`__ para que sea accesible en https://design.ros2.org/.
   * Debes realizar un fork al repositorio y enviar una pull request que detalle el diseño.

   Menciona el problema de ros2 relacionado (por ejemplo, ``Documento de diseño para la tarea ros2/ros2#<Id. de problema>``) en el pull request o en el mensaje de confirmación.
   Las instrucciones detalladas se encuentran en la página de `ROS 2 Contribute <https://design.ros2.org/contribute.html>`__.
   Los comentarios de diseño se realizarán directamente en el pull request.

Si se planea lanzar la tarea con una versión específica de ROS, esta información debe incluirse en el pull request.

Revisión del documento de diseño
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Una vez que el diseño está listo para su revisión, se debe abrir una pull request y se deben asignar los revisores apropiados.
Se recomienda incluir a los propietarios del proyecto:
mantenedores de todos los paquetes afectados (como se define en el campo de mantenedor ``package.xml``, consulte `REP-140 <https://www.ros.org/reps/rep-0140.html#maintainer-multiple-but-at-least-one>`__) - como revisores.

* Si el documento de diseño es complejo o los revisores tienen horarios conflictivos, se puede programar una reunión de revisión de diseño opcional.
   En este caso,

   **Antes de la reunión**

   * Envía una invitación a una reunión con al menos una semana de anticipación
   * Se recomienda una duración de la reunión de una hora
   * La invitación a la reunión debe enumerar todas las decisiones que se tomarán durante la revisión (decisiones que requieren la aprobación del mantenedor del paquete)
   * Asistentes requeridos en la reunión: revisores de pull request de diseño
       Asistentes opcionales: todos los ingenieros de OSRF, si corresponde

   **Durante la reunión**

   * El responsable de la tarea dirige la reunión, presenta tus ideas y gestiona los debates para garantizar que se llegue a un acuerdo a tiempo

   **Después de la reunión**

   * El propietario de la tarea debe enviar las notas de la reunión a todos los asistentes
   * Si se han planteado cuestiones menores sobre el diseño:

     * El propietario de la tarea debe actualizar el pull request del documento de diseño en función de los comentarios.
     * No se requiere revisión adicional

   * Si se han planteado cuestiones importantes sobre el diseño:

     * Es aceptable eliminar secciones para las que no hay un acuerdo claro
     * Las partes discutibles del diseño se pueden volver a enviar como una tarea separada en el futuro
     * Si eliminar las partes discutibles no es una opción, trabaja directamente con los propietarios del paquete para llegar a un acuerdo

* Una vez alcanzado el consenso:

   * Asegúrate de que el pull request `ros2/design <https://github.com/ros2/design/>`__ se haya fusionado, si corresponde
   * Actualiza y cierra el problema de GitHub asociado con esta tarea de diseño

Implementación
~~~~~~~~~~~~~~

Antes de comenzar, revise la sección `Pull requests`_ para conocer las mejores prácticas.

* Para cada repo a modificar:

   * Modifica el código, vaya al siguiente paso si terminaste o en intervalos regulares para hacer una copia de seguridad de su trabajo.
   * `Auto-revise <https://git-scm.com/book/en/v2/Git-Tools-Interactive-Staging>`__ tus cambios usando ``git add -i``.
   * Crea un nuevo commit firmada usando ``git commit -s``.

     * Una pull request debe contener confirmaciones mínimas semánticamente significativas (por ejemplo, no es aceptable una gran cantidad de commits de 1 línea).
       Crea nuevos commit de corrección mientras iteras según la retroalimentación u, opcionalmente, modifica los commits existentes usando ``git commit --amend`` si no desea crear un nuevo commit cada vez.
     * Cada commit debe tener un mensaje de commit correctamente escrito y significativo.
       Más instrucciones `aquí <https://chris.beams.io/posts/git-commit/>`__.
     * El movimiento de archivos debe realizarse en una confirmación separada; de lo contrario, es posible que git no realice un seguimiento preciso del historial del archivo.
     * La descripción del pull request o el mensaje de confirmación deben contener una referencia al problema de ros2 relacionado, por lo que se cierra automáticamente cuando se fusiona el pull request.
       Consulte este `doc <https://help.github.com/articles/closing-issues-using-keywords/>`__ para obtener más detalles.
     * Haga push de  los nuevos commits.

Revisión de código
~~~~~~~~~~~~~~~~~~

Una vez que el cambio esté listo para la revisión del código:

* Abre un pull request para cada repositorio modificado.

   * Recuerda seguir las mejores prácticas de `Pull requests`_.
   * Se puede usar `GitHub <https://hub.github.com/>`__  para crear pull request desde la línea de comandos.
   * Si se planea lanzar la tarea con una versión específica de ROS, esta información debe incluirse en cada pull request.

* Los propietarios del paquete que revisaron el documento de diseño deben mencionarse en el pull request.
* SLO de revisión de código: aunque revisar las solicitudes de incorporación de cambios es el mejor esfuerzo,
   es útil que los revisores comenten sobre los pull request dentro de una semana y que
   los autores respondan a los comentarios dentro de una semana, para que no haya pérdida de contexto.
* Como siempre itera sobre los comentarios, modifica y actualiza la rama de desarrollo según sea necesario.
* Una vez que se apruebe el PR, los mantenedores del paquete fusionarán los cambios.


Introducción a la construcción de una granja
--------------------------------------------

La granja de compilación se encuentra en `ci.ros2.org <https://ci.ros2.org/>`__.

Todas las noches ejecutamos trabajos nocturnos que compilan y ejecutan todas las pruebas en varios escenarios en varias plataformas.
Además, probamos todos los pull request en estas plataformas antes de fusionarlos.

Este es el conjunto actual de plataformas y arquitecturas de destino, aunque evoluciona con el tiempo:


* Ubuntu 22.04 Jammy

   * amd64
   * aarch64

* Windows 10

   * amd64

Hay varias categorías de trabajos en buildfarm:


* trabajos manuales (activados manualmente por los desarrolladores):

   * ci_linux: construir + probar el código en Ubuntu Xenial
   * ci_linux-aarch64: compilar + probar el código en Ubuntu Xenial en una máquina ARM de 64 bits (aarch64)
   * ci_linux_coverage: compilación + prueba + generación de cobertura de prueba
   * ci_windows: construye + prueba el código en Windows 10
   * ci_launcher: activa todos los trabajos enumerados anteriormente

* nocturno (ejecutar todas las noches):

   * Depurar: construir + probar el código con CMAKE_BUILD_TYPE=Depurar

     * nightly_linux_debug
     * nightly_linux-aarch64_debug
     * nightly_win_deb

   * Lanzamiento: construye + prueba el código con CMAKE_BUILD_TYPE=Release

     * nightly_linux_release
     * nightly_linux-aarch64_release
     * nightly_win_rel

   * Repetido: construye y luego ejecute cada prueba hasta 20 veces o hasta que falle (también conocido como cazador de fragilidad)

     * nightly_linux_repeated
     * nightly_linux-aarch64_repeated
     * nightly_win_rep

   * Cobertura:

     * nightly_linux_coverage: construye + prueba el código + analiza la cobertura para c/c++ y python

       * los resultados se exportan como informe de cobertura


* empaquetado (ejecutar todas las noches; el resultado se agrupa en un archivo):

   * packaging_linux
   * packaging_windows

Dos granjas de compilación adicionales respaldan el ecosistema ROS / ROS 2 al proporcionar la creación de fuente y
paquetes binarios, integración continua, pruebas y análisis.

Para obtener detalles, preguntas frecuentes y resolución de problemas, consulta :doc:`build farms <Build-Farms>`.

Nota sobre la ejecución de cobertura
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Los paquetes de ROS 2 están organizados de manera que el código de prueba para un paquete dado no solo está contenido dentro del paquete, sino que también podría estar presente en un paquete diferente.
En otras palabras: los paquetes pueden ejercer código perteneciente a otros paquetes durante la fase de prueba.

Para lograr la tasa de cobertura alcanzada por todo el código disponible en los paquetes principales de ROS 2, se recomienda ejecutar compilaciones utilizando un conjunto fijo de repositorios propuestos.
Ese conjunto se define en los parámetros predeterminados de trabajos de cobertura en Jenkins.


.. _read-coverage-report:

Cómo leer la tasa de cobertura del informe buildfarm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Para ver el informe de cobertura de un paquete determinado:

* Cuando finalice la compilación ``ci_linux_coverage``, haga clic en ``Coverage Report``
* Desplázate hacia abajo hasta la tabla ``overage Breakdown by Package``
* En la tabla, mira la primera columna llamada "Name"

Los informes de cobertura en buildfarm incluyen todos los paquetes que se usaron en el espacio de trabajo de ROS.
El informe de cobertura incluye diferentes rutas correspondientes a un mismo paquete:

* Nombre entradas con la forma: ``src.*.<repository_name>.<package_name>.*``
   Estos corresponden a las ejecuciones de pruebas unitarias disponibles en un paquete contra su propio código fuente
* Nombre entradas con la forma: ``build.<repository_name>.<package_name>.*``
   Estos corresponden a las ejecuciones de prueba unitarias disponibles en un paquete contra sus archivos generados en el momento de la construcción o configuración.
* Nombre entradas con la forma: ``install.<package_name>.*``
   Estos corresponden a las pruebas de sistema/integración provenientes de las ejecuciones de prueba de otros paquetes.

.. _calculate-coverage-rate:

Cómo calcular la tasa de cobertura del informe buildfarm
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Obtén la tasa de cobertura de unidad combinada utilizando el script automático:

  * Desde la compilación ci_linux_coverage Jenkins, copia la URL de la compilación
  * Descarga el script `get_coverage_ros2_pkg <https://raw.githubusercontent.com/ros2/ci/master/tools/get_coverage_ros2_pkg.py>`__
  * Ejecuta el script: ``./get_coverage_ros2_pkg.py <jenkins_build_url> <ros2_package_name>`` (`README <https://github.com/ros2/ci/blob/master/tools/README.md>`__)
  * Toma los resultados de la línea final "Prueba de unidad combinada" en la salida del script

Alternativa: obten la tasa de cobertura unitaria combinada del informe de cobertura (requiere cálculo manual):

* Cuando finalices la compilación de ci_linux_coverage, haz clic en ``Informe de cobertura de Cobertura``
* Desplázate hacia abajo hasta la tabla ``Desglose de cobertura por paquete``
* En la tabla, debajo de la primera columna "Name", busque (donde <package_name> es su paquete bajo prueba):

   * todos los directorios bajo el patrón ``src.*.<repository_name>.<package_name>.*`` toma los dos valores absolutos en la columna "Lines".
   * todos los directorios bajo el patrón ``build/.<repository_name>.*`` toma los dos valores absolutos en la columna "Lines".

* Con la selección anterior: para cada celda, el primer valor son las líneas probadas y el segundo el total de líneas de código.
   Agrega todas las filas para obtener el total de líneas probadas y el total de líneas de código bajo prueba.
   Divide para obtener la tasa de cobertura.

.. _medir-cobertura-localmente:

Cómo medir la cobertura localmente usando lcov (Ubuntu)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Para medir la cobertura en su propia máquina, instala ``lcov``.

 .. code-block:: bash

     sudo apt install -y lcov

El resto de esta sección asume que estás trabajando desde tu espacio de trabajo colcon.
Compila en depuración con banderas de cobertura.
Siéntete libre de usar banderas colcon para apuntar a paquetes específicos.

.. code-block:: bash

     colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS} --coverage" -DCMAKE_C_FLAGS="${CMAKE_C_FLAGS} --coverage"

``lcov`` requiere una línea de base inicial, que puede producir con el siguiente comando.
Actualiza la ubicación del archivo de salida según sus necesidades.

.. code-block:: bash

     lcov --no-external --capture --initial --directory . --output-file ~/ros2_base.info

Ejecuta pruebas para los paquetes que son importantes para sus medidas de cobertura.
Por ejemplo, si se mide ``rclcpp`` también con ``test_rclcpp``

.. code-block:: bash

     colcon test --packages-select rclcpp test_rclcpp

Captura los resultados de lcov con un comando similar esta vez dejando caer el indicador ``--initial``.

.. bloque de código:: bash

      lcov --no-externo --capturar --directorio . --archivo de salida ~/ros2.info

Combina los archivos de rastreo .info:

.. code-block:: bash

     lcov --add-tracefile ~/ros2_base.info --add-tracefile ~/ros2.info --output-file ~/ros2_coverage.info

Genera html para facilitar la visualización y la anotación de las líneas cubiertas.

.. code-block:: bash

    mkdir -p coverage
    genhtml ~/ros2_coverage.info --output-directory coverage
