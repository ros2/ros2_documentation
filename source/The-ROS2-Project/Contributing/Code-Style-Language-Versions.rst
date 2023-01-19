.. redirect-from::

  Contributing/Code-Style-Language-Versions

.. _CodeStyle:

Estilo de código y versiones de lenguaje
========================================

.. contents:: Tabla de Contenido
   :depth: 2
   :local:

Para lograr un producto de apariencia consistente, todos seguiremos para cada lenguaje de programación guías de estilo (de ser posible) que ya estén definidas externamente.
Para otras cosas, como el diseño de paquetes o el diseño de la documentación, necesitaremos crear nuestras propias pautas, basándonos en los estilos actuales y populares que se usan ahora.

Además, siempre que sea posible, los desarrolladores deben usar herramientas integradas que les permitan verificar que se sigan estas pautas en sus editores.
Por ejemplo, todos deberían tener un verificador PEP8 integrado en su editor para reducir las iteraciones de revisión relacionadas con el estilo.

Además, cuando sea posible, los paquetes deben verificar el estilo como parte de sus pruebas unitarias para ayudar con la detección automática de problemas de estilo (ver `ament_lint_auto <https://github.com/ament/ament_lint/blob/{REPOS_FILE_BRANCH}/ament_lint_auto/doc/index.rst>`__).


C
-

Estándar
^^^^^^^^

Se apunta a C99.

Estilo
^^^^^^

Usaremos `Python's PEP7 <https://www.python.org/dev/peps/pep-0007/>`__ para nuestra guía de estilo C, con algunas modificaciones y adiciones:

* Apuntamos a C99, ya que no necesitamos admitir C89 (como recomienda PEP7)

  * justificación: entre otras cosas, nos permite usar comentarios de estilo ``//`` y ``/* */``
  * justificación: C99 es bastante omnipresente ahora

* Se permiten comentarios estilo C++ ``//``
* (opcional) Coloque siempre los literales en el lado izquierdo de los operadores de comparación, por ejemplo, ``0 == ret`` en lugar de ``ret == 0``

  * justificación: ``ret == 0`` puede convertirse en fácilmente ``ret = 0`` por accidente
  * opcional porque al usar ``-Wall`` (o equivalente) los compiladores modernos le avisarán cuando esto suceda

Todas las siguientes modificaciones solo se aplican si no se esta escribiendo módulos de Python:

* No use ``Py_`` como prefijo para todo

  * en su lugar, use una versión CamelCase del nombre del paquete u otro prefijo apropiado

* las cosas sobre documentación de strings no se aplica

Podemos usar el módulo de python `pep7 <https://github.com/mike-perdide/pep7>`__  para verificación de estilo. La integración del editor parece escasa, es posible que debamos analizar la verificación automática de C con más detalle.

C++
---

Estándar
^^^^^^^^

{DISTRO_TITLE} toma como referencia a C++17.

Estilo
^^^^^^


Usaremos la Guía de estilo de `Google C++ Style Guide <https://google.github.io/styleguide/cppguide.html>`__, con algunas modificaciones:

Longitud de la línea
~~~~~~~~~~~~~~~~~~~~

* Nuestra longitud máxima de línea es de 100 caracteres.

Extensiones de archivo
~~~~~~~~~~~~~~~~~~~~~~

* Los archivos de encabezado deben usar la extensión .hpp.

  * justificación: permitir que las herramientas determinen el contenido de los archivos, C++ o C.

* Los archivos de implementación deben usar la extensión .cpp.

  * justificación: permitir que las herramientas determinen el contenido de los archivos, C++ o C.

Nomenclatura de variables
~~~~~~~~~~~~~~~~~~~~~~~~~

* Para variables globales, use minúsculas con guiones bajos con el prefijo ``g_``

  * justificación: mantener la coherencia entre mayúsculas y minúsculas en todo el proyecto
  * justificación: es fácil saber el alcance de una variable de un vistazo
  * coherencia entre lenguajes

Denominación de funciones y métodos
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* La guía de estilo de Google dice ``CamelCase``, pero también se permite el estilo ``snake_case`` de la biblioteca estándar de C++

  * justificación: los paquetes principales de ROS 2 actualmente usan ``snake_case``

    * razón: ya sea un descuido histórico o una preferencia personal que no fue verificada por el linter
    * razón para no cambiar: cambiar retroactivamente sería demasiado disruptivo
  * Otras Consideraciones:

    * ``cpplint.py`` no verifica este caso (difícil de hacer cumplir excepto con revisión)
    * ``snake_case`` puede dar como resultado una mayor coherencia entre lenguajes
  * orientación específica:

    * para proyectos existentes, se prefiere el estilo existente
    * para nuevos proyectos, cualquiera de los dos es aceptable, pero se recomienda una preferencia por la coincidencia de proyectos existentes relacionados
    * la decisión final es siempre discreción del desarrollador

      * casos especiales como punteros de función, tipos invocables, etc. pueden requerir doblar las reglas
    * Tenga en cuenta que las clases aún deben usar ``CamelCase`` por defecto

Control de acceso
~~~~~~~~~~~~~~~~~

* Elimina el requisito de que todos los miembros de la clase sean privados y, por lo tanto, requieran accessors

  * justificación: esto es demasiado restrictivo para el diseño de API de usuario
  * debemos preferir miembros privados, solo haciéndolos públicos cuando sean necesarios
  * debemos considerar el uso de accessors antes de elegir permitir el acceso directo a los miembros
  * debemos tener una buena razón para permitir el acceso directo de los miembros, que sea conveniente para nosotros no es razón suficiente.

Excepciones
~~~~~~~~~~~

* Se permiten excepciones

  * justificación: esta es una nueva base de código, por lo que el argumento heredado no se aplica a nosotros
  * justificación: para las API orientadas al usuario, es más idiomático C++ tener excepciones
  * Las excepciones en los destructores deben evitarse explícitamente

* Debemos considerar evitar Excepciones si tenemos la intención de envolver la API resultante en C

  * justificación: hará que sea más fácil envolver en C
  * justificación: la mayoría de nuestras dependencias en el código que pretendemos envolver en C no usan excepciones de todos modos

Objetos similares a funciones
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Sin restricciones en Lambda's o ``std::function`` o ``std::bind``

Boost
~~~~~~~~

* Boost debe evitarse a menos que sea absolutamente necesario.

Comentarios y comentarios del documento
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Utilice los comentarios ``///`` y ``/** */`` para propósitos de *documentación* y comentarios de estilo ``//`` para notas y comentarios generales

  * Los comentarios de clase y función deben usar comentarios de estilo ``///`` y ``/** */``
  * justificación: se recomiendan para Doxygen y Sphinx en C/C++
  * justificación: mezclar ``/* */`` y ``//`` es conveniente para comentar bloques de código que contengan comentarios
  * Las descripciones de cómo funciona el código o las notas dentro de las clases y funciones deben usar comentarios de estilo ``//``

Alineación de sintaxis de puntero
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Use ``char * c;`` en lugar de ``char* c;`` o ``char *c;`` debido a este escenario ``char* c, *d, *e;``

Palabras clave de privacidad de clase
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* No coloque 1 espacio antes de ``public:``, ``private:`` o ``protected:``, es más consistente que todas las sangrías sean múltiplos de 2

  * justificación: a la mayoría de los editores no les gustan las sangrías que no son un múltiplo del tamaño de la pestaña (suave)
  * Usa cero espacios antes de ``public:``, ``private:``, o ``protected:``, o 2 espacios
  * Si usa 2 espacios antes, indenta otras declaraciones de clase con 2 espacios adicionales
  * Se prefiere cero espacios, es decir, ``public:``, ``private:`` o ``protected:`` en la misma columna que la clase

Plantillas anidadas
~~~~~~~~~~~~~~~~~~~

* Nunca agregue espacios en blanco a las plantillas anidadas

  * Se prefiere ``set<list<string>>`` (característica de C++11 ) a ``set<list<string> >`` o ``set< list<string> >``

Paréntesis abiertos Versus Cuddled
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* Usa paréntesis en una nueva linea para ``function``, ``class``, ``enum``, y ``struct``, pero paréntesis en la misma linea para ``if``, ``else``, ``while``, ``for``, etc...

  * Excepción: cuando un ``if`` (o ``while``, etc.)  es lo suficiente largo que no quepa en linea, entonces usar nueva linea (i.e., don't cuddle).

* Cuando una llamada a una función no quepa en una linea, rompe la linea (no entre argumentos) y empieza en la proxima linea con indentación de 2 espacios.  Si se tienen más argumentos continua con la indentación de 2 espacios en las lineas siguientes .  (Nota que la  `Guía de estilos de Google <https://google.github.io/styleguide/cppguide.html#Function_Calls>`__ se autocontradice en este punto.)

  * Lo mismo se aplica a ``if`` (y ``while``, etc.) si son demasiado largas para caber en una linea.

Ejemplos
~~~~~~~~

Esto esta OK:

.. code-block:: c++

   int main(int argc, char **argv)
   {
     if (condition) {
       return 0;
     } else {
       return 1;
     }
   }

   if (this && that || both) {
     ...
   }

   // Long condition; open brace
   if (
     this && that || both && this && that || both && this && that || both && this && that)
   {
     ...
   }

   // Short function call
   call_func(foo, bar);

   // Long function call; wrap at the open parenthesis
   call_func(
     foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar,
     foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar, foo, bar);

   // Very long function argument; separate it for readability
   call_func(
     bang,
     fooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo,
     bar, bat);

Esto **no** esta bien:

.. code-block:: c++

   int main(int argc, char **argv) {
     return 0;
   }

   if (this &&
       that ||
       both) {
     ...
   }


Usa corchetes abierto en lugar de usar sangria de manera excesiva, p.ej. para distinguir el código del constructor de las listas de inicializadores del constructor

Esto esta OK:

.. code-block:: c++

   ReturnType LongClassName::ReallyReallyReallyLongFunctionName(
     Type par_name1,  // 2 space indent
     Type par_name2,
     Type par_name3)
   {
     DoSomething();  // 2 space indent
     ...
   }

   MyClass::MyClass(int var)
   : some_var_(var),
     some_other_var_(var + 1)
   {
     ...
     DoSomething();
     ...
   }

Esto **no** está bien, incluso es extraño (¿a la manera de Google?):

.. code-block:: c++

   ReturnType LongClassName::ReallyReallyReallyLongFunctionName(
       Type par_name1,  // 4 space indent
       Type par_name2,
       Type par_name3) {
     DoSomething();  // 2 space indent
     ...
   }

   MyClass::MyClass(int var)
       : some_var_(var),             // 4 space indent
         some_other_var_(var + 1) {  // lined up
     ...
     DoSomething();
     ...
   }

Linters
~~~~~~~

El estilo fue verificado con una combinación de `cpplint.py <https://github.com/google/styleguide>`__  de Google y `uncrustify <https://github.com/uncrustify/uncrustify>`__.

Proporcionamos herramientas de linea de comandos con las configuraciones personalizada:

* `ament_clang_format <https://github.com/ament/ament_lint/blob/{REPOS_FILE_BRANCH}/ament_clang_format/doc/index.rst>`__: `configuration <https://github.com/ament/ament_lint/blob/{REPOS_FILE_BRANCH}/ament_clang_format/ament_clang_format/configuration/.clang-format>`__
* `ament_cpplint <https://github.com/ament/ament_lint/blob/{REPOS_FILE_BRANCH}/ament_cpplint/doc/index.rst>`__
* `ament_uncrustify <https://github.com/ament/ament_lint/blob/{REPOS_FILE_BRANCH}/ament_uncrustify/doc/index.rst>`__: `configuration <https://github.com/ament/ament_lint/blob/{REPOS_FILE_BRANCH}/ament_uncrustify/ament_uncrustify/configuration/ament_code_style.cfg>`__

Algunos correctores de estilo como ament_uncrustify y ament_clang_format tienen la opción ``--reformat`` para aplicar los cambios.

También ejecutamos otras herramientas para detectar y eliminar tantas advertencias como sea posible.
Aquí hay una lista no exhaustiva de cosas adicionales que tratamos de hacer en todos nuestros paquetes:

* usar banderas del compilador como ``-Wall -Wextra -Wpedantic``
* ejecutar análisis de código estático como ``cppcheck``, que hemos integrado en `ament_cppcheck <https://github.com/ament/ament_lint/blob/{REPOS_FILE_BRANCH}/ament_cppcheck/doc/index.rst>`__.

Python
------

Versión
^^^^^^^

Apuntaremos a Python 3 para nuestro desarrollo.

Estilo
^^^^^^

Usaremos las `directrices de PEP8 <https://www.python.org/dev/peps/pep-0008/>`__ para el formato del código.

Elegimos la siguiente regla más precisa donde PEP 8 deja cierta libertad:

* `Permitimos hasta 100 caracteres por línea (quinto párrafo) <https://www.python.org/dev/peps/pep-0008/#maximum-line-length>`__.
* `Elegimos comillas simples sobre comillas dobles siempre que no sea necesario escapar <https://www.python.org/dev/peps/pep-0008/#string-quotes>`__.
* `Preferimos sangrías colgantes para las líneas de continuación <https://www.python.org/dev/peps/pep-0008/#indentation>`__.

Las herramientas como el paquete de Python ``(ament_)pycodestyle`` deben usarse en la integración de pruebas unitarias y/o editores para comprobar el estilo del código de Python.

La configuración de pycodestyle utilizada en el linter está `aquí <https://github.com/ament/ament_lint/blob/{REPOS_FILE_BRANCH}/ament_pycodestyle/ament_pycodestyle/configuration/ament_pycodestyle.ini>`__.

Integración con editores:

* atom: https://atom.io/packages/linter-pycodestyle
* emacs: https://www.emacswiki.org/emacs/PythonProgrammingInEmacs
* Sublime Text: https://sublime.wbond.net/packages/SublimeLinter-flake8
* vim: https://github.com/nvie/vim-flake8

CMake
-----

Versión
^^^^^^^

Apuntaremos a CMake 3.8.

Estilo
^^^^^^

Dado que no existe una guía de estilo de CMake, definiremos la nuestra:

* Usa nombres de comando en minúsculas (``find_package``, no ``FIND_PACKAGE``).
* Usa identificadores ``snake_case`` (variables, funciones, macros).
* Utiliza los comandos vacíos ``else()`` y ``end...()``.
* Sin espacios en blanco antes de ``(``\ 's.
* Usa dos espacios de sangría, no use tabulaciones.
* No uses sangría alineada para parámetros de invocaciones de macros de varias líneas. Use dos espacios solamente.
* Se Preferiré funciones con ``set(PARENT_SCOPE)`` a macros.
* Al usar macros prefija las variables locales con ``_`` o un prefijo razonable.

Markdown / reStructured Text / docblocks
----------------------------------------

Estilo
^^^^^^

Las siguientes reglas para dar formato al texto están destinadas a aumentar la legibilidad, así como el control de versiones.

* *[.md, .rst solamente]* Cada título de sección debe estar precedido por una línea vacía y seguido por una línea vacía.

   * Justificación: Es rápido obtener una visión general de la estructura al examinar el documento.

* *[Solo .rst]* En el texto reStructured, los encabezados deben seguir la jerarquía descrita en la `guía de estilo de Sphinx <https://documentation-style-guide-sphinx.readthedocs.io/en/latest/style-guide.html#headings>`__:

   * ``#`` con línea superior (solo una vez, se usa para el título del documento)
   * ``*`` con sobrelínea
   * ``=``
   * ``-``
   * ``^``
   * ``"``
   * Justificación: una jerarquía coherente acelera la obtención de una idea sobre el nivel de anidamiento al examinar el documento.

* *[Solo .md]* En Markdown, los encabezados deben seguir el estilo ATX descrito en la `documentación de sintaxis de Markdown <https://daringfireball.net/projects/markdown/syntax#header>`__

   * Los encabezados de estilo ATX usan de 1 a 6 caracteres hash (``#``) al comienzo de la línea para indicar los niveles de encabezado 1-6.
   * Se debe usar un espacio entre los hash y el título del encabezado (como ``# Heading 1``) para que sea más fácil separarlos visualmente.
   * La justificación de la preferencia del estilo ATX proviene de la `guía de estilo de Google Markdown <https://github.com/google/styleguide/blob/gh-pages/docguide/style.md#atx-style-headings>`__
   * Justificación: los encabezados de estilo ATX son más fáciles de buscar y mantener, y hacen que los dos primeros niveles de encabezado sean coherentes con los otros niveles.

* *[cualquiera]* Cada oración debe comenzar en una nueva línea.

   * Justificación: para párrafos más largos, un solo cambio al principio hace que la diferencia sea ilegible, ya que continúa a lo largo de todo el párrafo.

* *[cualquiera]* Opcionalmente, cada oración se puede envolver para que cada línea sea corta.
* *[cualquiera]* Las líneas no deben tener ningún espacio en blanco al final.
* *[.md, .rst solamente]* Un bloque de código debe estar precedido y seguido por una línea vacía.

   * Justificación: los espacios en blanco son significativos solo directamente antes y después de los bloques de código delimitados.
     Seguir estas instrucciones asegurará que el resaltado funcione de manera adecuada y consistente.

* *[.md, .rst solamente]* Un bloque de código debe especificar una sintaxis (por ejemplo, ``bash``).
