.. redirect-from::

    Real-Time-Programming
    Tutorials/Real-Time-Programming

Understanding real-time programming
===================================

.. contents:: Tabla de contenidos
   :depth: 2
   :local:

Historial
---------

La computación en tiempo real es una característica clave de muchos sistemas robóticos, en particular aplicaciones de misión crítica y seguridad, como vehículos autónomos, naves espaciales y fabricación industrial.
Se está diseñando y creando prototipos de ROS 2 teniendo en cuenta las limitaciones de rendimiento en tiempo real, ya que este es un requisito que no se consideró en las primeras etapas de ROS 1 y ahora es intratable refactorizar ROS 1 para que sea amigable en tiempo real.

`Este documento <https://design.ros2.org/articles/realtime_background.html>`__ describe los requisitos de la computación en tiempo real y las mejores prácticas para los ingenieros de software. En resumen:

Para hacer un sistema informático en tiempo real, nuestro bucle en tiempo real debe actualizarse periódicamente para cumplir con los plazos.
Solo podemos tolerar un pequeño margen de error en estos plazos (nuestra fluctuación máxima permitida).
Para hacer esto, debemos evitar operaciones no deterministas en la ruta de ejecución, cosas como: eventos de fallo de página, asignación/desasignación de memoria dinámica y primitivas de sincronización que se bloquean indefinidamente.

Un ejemplo clásico de un problema de controles comúnmente resuelto por computación en tiempo real es equilibrar un péndulo invertido <https://en.wikipedia.org/wiki/Inverted_pendulum>`__.
Si el controlador se bloqueara durante un período de tiempo inesperadamente largo, el péndulo se caería o se volvería inestable.
Pero si el controlador se actualiza de manera confiable a una velocidad más rápida que la que puede operar el motor que controla el péndulo, el péndulo se adaptará con éxito y reaccionará a los datos del sensor para equilibrar el péndulo.

Ahora que sabe todo acerca de la computación en tiempo real, ¡probemos una demo!

Instala y ejecuta la demo
-------------------------

La demostración en tiempo real se escribió pensando en los sistemas operativos Linux, ya que muchos miembros de la comunidad ROS que realizan computación en tiempo real usan Xenomai o RT_PREEMPT como sus soluciones en tiempo real.
Dado que muchas de las operaciones realizadas en la demostración para optimizar el rendimiento son específicas del sistema operativo, la demostración solo se compila y ejecuta en sistemas Linux.
**Entonces, si eres un usuario de OSX o Windows, ¡no intentes esta parte!**

Además, esto debe construirse desde la fuente utilizando una API DDS estática. **Actualmente, la única implementación admitida es Connext**.

Primero, sigue las instrucciones para compilar ROS 2 :doc:`from source <../../Installation/Alternatives/Ubuntu-Development-Setup>` utilizando Connext DDS como middleware.

Ejecuta los test
^^^^^^^^^^^^^^^^

**Antes de ejecutar, asegúrate de tener al menos 8 Gb de RAM libres. Con el bloqueo de la memoria, el intercambio ya no funcionará.**

Ejecuta ``source`` con el archivo de setup.

Ejecuta el binario de la demo y redirija la salida. Es posible que quieras usar ``sudo`` en caso de que obtengas un error de permiso:

.. code-block:: bash

   pendulum_demo > output.txt

¿Qué diablos acaba de pasar?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Primero, aunque redirigió stdout, verás algunos resultados en la consola (desde stderr):

::

   mlockall falló: no se puede asignar memoria
   No se pudo bloquear toda la memoria virtual almacenada en caché.
   Se registrarán los fallos de página de la lectura de páginas que aún no se han asignado a la RAM.

Después de la etapa de inicialización del programa demo, intentará bloquear toda la memoria caché en la RAM y evitar futuras asignaciones de memoria dinámica utilizando ``mlockall``.
Esto es para evitar que los fallos de página carguen mucha memoria nueva en la RAM.
(Consulte `el artículo de diseño en tiempo real <https://design.ros2.org/articles/realtime_background.html#memory-management>`__ para obtener más información).

La demo continuará como de costumbre cuando esto ocurra.
En la parte inferior del archivo output.txt generado por la demo, verás la cantidad de fallas de página encontradas durante la ejecución:

::

   Estadísticas de prueba:
      - Faltas de página menores: 20
      - Principales fallos de página: 0

Si queremos que esos fallos de página desaparezcan, tendremos que...

Ajustar permisos para bloqueo de memoria
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Agregue a ``/etc/security/limits.conf`` (como sudo):

::

   <tu usuario>    -   memlock   <limite en kB>

Un límite de ``-1`` es ilimitado.
Si eliges esto, es posible que debas acompañarlo con ``ulimit -l unlimited`` después de editar el archivo.

Después de guardar el archivo, cierra la sesión y vuelveiniciarla.
Luego vuelve a ejecutar la invocación ``pendulum_demo``.

Verás cero errores de página en tu archivo de salida o un error que indica que se detectó una excepción bad_alloc.
Si esto sucediera, no tenía suficiente memoria libre disponible para bloquear la memoria asignada para el proceso en la RAM.
¡Tendrás que instalar más RAM en su ordenador para ver cero fallas de página!

Resumen de salida
^^^^^^^^^^^^^^^^^

Para ver más resultados, tenemos que ejecutar el nodo ``pendulum_logger``.

En un shell con su fuente ``install/setup.bash``, ejecuta:

.. code-block:: bash

   pendulum_logger


Deberías ver el mensaje de salida:

::

   Nodo registrador inicializado.

En otro shell con setup.bash de origen, ejecuta ``pendulum_demo`` de nuevo.

Tan pronto como se inicie este ejecutable, deberías ver el otro shell imprimiendo constantemente la salida:

::

   Ángulo motor comandado: 1.570796
   Ángulo real del motor: 1.570796
   Latencia media: 210144.000000 ns
   Latencia mínima: 4805 ns
   Latencia máxima: 578137 ns
   Errores de página menores durante la ejecución: 0
   Principales fallas de página durante la ejecución: 0

La demostración controla una simulación de péndulo invertido muy simple.
La simulación del péndulo calcula su posición en su propio hilo.
Un nodo ROS simula un sensor codificador de motor para el péndulo y publica su posición.
Otro nodo ROS actúa como un controlador PID simple y calcula el siguiente mensaje de comando.

El nodo registrador imprime periódicamente el estado del péndulo y las estadísticas de rendimiento del tiempo de ejecución de la demostración durante su fase de ejecución.

Después de que finalice ``pendulum_demo``, tendrás que pulsar CTRL-C para salir del nodo del registrador.

Latencia
^^^^^^^^

En la ejecución de ``pendulum_demo``, verás las estadísticas finales recopiladas para la demostración:

::

   Estadísticas de prueba:
      - Faltas de página menores: 0
      - Principales fallos de página: 0
      Latencia (tiempo después de que se perdió la fecha límite):
        - Min: 3354 ns
        - Máx: 2752187 ns
        - Media: 19871.8 ns
        - Desviación estándar: 1.35819e+08

    PendulumMotor recibió 985 mensajes
    PendulumController recibió 987 mensajes

Los campos de latencia te muestran la latencia mínima, máxima y promedio del bucle de actualización en nanosegundos.
Aquí, latencia significa la cantidad de tiempo después de que se esperaba que ocurriera la actualización.

Los requisitos de un sistema en tiempo real dependen de la aplicación, pero supongamos que en esta demostración tenemos un ciclo de actualización de 1 kHz (1 milisegundo) y nuestro objetivo es una latencia máxima permitida del 5 % de nuestro período de actualización.

Entonces, nuestra latencia promedio fue realmente buena en esta ejecución, pero la latencia máxima fue inaceptable porque en realidad excedió nuestro ciclo de actualización. ¿Qué sucedió?

Es posible que suframos de un planificador no determinista.
Si estás ejecutando un sistema Vanilla Linux y no tienes instalado el kernel RT_PREEMPT, probablemente no podrás cumplir con el objetivo en tiempo real que nos fijamos, porque el programador de Linux no te permitirá arbitrariamente adelantarse a los subprocesos a nivel de usuario.

Consulta el `artículo de diseño en tiempo real <https://design.ros2.org/articles/realtime_background.html#multithreaded-programming-and-synchronization>`__ para obtener más información.

La demostración intenta establecer la prioridad del programador y del subproceso de la demostración para que sea adecuada para el rendimiento en tiempo real.
Si esta operación falla, verás un mensaje de error: "No se pudo establecer la política y la prioridad de programación: operación no permitida".
Puedes obtener un rendimiento ligeramente mejor siguiendo las instrucciones de la siguiente sección:

Configuración de permisos para el programador
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Agrega a ``/etc/security/limits.conf`` (como sudo):

::

   <tu usuario>    -   rtprio   98

El rango del campo rtprio (prioridad en tiempo real) es 0-99.
Sin embargo, NO establezcas el límite en 99 porque tus procesos podrían interferir con procesos importantes del sistema que se ejecutan con máxima prioridad (por ejemplo, vigilancia).
Esta demo intentará ejecutar el lazo de control con prioridad 98.

Resultados gráficos
^^^^^^^^^^^^^^^^^^^

Puedes graficar las estadísticas de latencia y fallo de página que se recopilan en esta demostración después de que se ejecuta la demostración.

Debido a que el código ha sido instrumentado con `rttest <https://github.com/ros2/rttest>`__, hay argumentos de línea de comando útiles disponibles:

+---------+---------------------------------------------------------------------------------------+-------------------+
| Comando | Descripción                                                                           | Valor por defecto |
+---------+---------------------------------------------------------------------------------------+-------------------+
| -i      | Especifica cuántas iteraciones ejecutar el ciclo en tiempo real                       | 1000              |
+---------+---------------------------------------------------------------------------------------+-------------------+
| -u      | Especifica el período de actualización con la unidad predeterminada en microsegundos. | 1ms               |
|         |                                                                                       |                   |
|         | Usa el sufijo "s" para segundos, "ms" para milisegundos,                              |                   |
|         |                                                                                       |                   |
|         | "us" para microsegundos, y "ns" para nanosegundos.                                    |                   |
+---------+---------------------------------------------------------------------------------------+-------------------+
| -f      | Especifica el nombre del archivo para escribir los datos recopilados.                 |                   |
+---------+---------------------------------------------------------------------------------------+-------------------+

Vuelve a ejecutar la demo con un nombre de archivo para guardar los resultados:

.. code-block:: bash

   pendulum_demo -f pendulum_demo_results

Luego ejecuta el script ``rttest_plot`` en el archivo resultante:

.. code-block:: bash

   rttest_plot pendulum_demo_results

Este script producirá tres archivos:

::

   pendulum_demo_results_plot_latency.svg
   pendulum_demo_results_plot_majflts.svg
   pendulum_demo_results_plot_minflts.svg

Puedes ver estos gráficos en un visor de imágenes de su elección.
