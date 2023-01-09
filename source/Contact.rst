.. _Help:

Contacto
=========

.. _Using ROS Answers:

Soporte
--------

Diferentes tipos de preguntas o discusiones corresponden a diferentes vias de comunicación;
asegúrate de revisar las descripciones a continuación para asegurarte de escoger la correcta.

¿Necesitas ayuda solucionando los problemas de tu sistema?
Primero, busca si en `ROS Answers <https://answers.ros.org>`__ existen problemas similares al tuyo y comprueba si las soluciones planteadas funcionan en tu caso.

De lo contrario, crea una nueva pregunta en `ROS Answers <https://answers.ros.org>`__.
Asegúrate de añadir las etiquetas correspondientes, o al menos la etiqueta de ``ros2`` y su distribución correspondiente, por ejemplo ``humble``.
Si tu pregunta esta relacionada con la documentación, añade la etiqueta ``docs``, o mas específicamente, ``tutorials``.

Contribuyendo a soporte
^^^^^^^^^^^^^^^^^^^^^^^^
Los usuarios de ROS 2 provienen de una amplia gama de antecedentes técnicos, usan una variedad de sistemas operativos diferentes y no necesariamente tienen experiencia previa con ROS (1 o 2).
Por lo tanto, es importante que los usuarios con cierta cantidad de experiencia aporten soporte.

Si ves un problema en `ROS Answers <https://answers.ros.org/questions/tags:ros2/>`__ y es similiar a algo en lo que has tenido experiencia, porfavor considera brindar algunos consejos que te fueron útiles en tu situación.
No te preocupes si no estas seguro de que tu respuesta es correcta.
Simplemente comparte tu solución, y los miembros de la comunidad intervendrán si es necesario.

Issues
-------

Si identificas bugs, tienes sugerencias de mejora, o tienes una pregunta especifica a un paquete en concreto, puedes abrir un issue en GitHub.

Por ejemplo, si estas siguiendo los :doc:`Tutoriales aca <Tutorials>` Y te encuentras con una instruccion que no funciona en tu sistema,
puedes abrir un issue en el siguiente repositorio: `ros2_documentation <https://github.com/ros2/ros2_documentation>`__ .

Puedes buscar por repositorios individuales de ROS2 en el siguiente enlace de `GitHub <https://github.com/ros2>`__.

Antes de abrir un issue, asegúrate que otros usuarios no hayan reportado un problema similar, lo puedes comprobar buscando las organizaciones encargadas de los repositorios: `ejemplo <https://github.com/search?q=user%3Aros2+user%3Aament+turtlesim&type=Issues>`__.

A continuación, revisa si en `ROS Answers <https://answers.ros.org/>`__ alguien ha preguntado o reportado un problema similar.

Si no ha sido reportado, siéntete libre de abrir un issue en el repositorio correcto.
Si no esta claro en donde debes iniciar el issue, hazlo en el siguiente repo `siguiente repo <https://github.com/ros2/ros2/issues>`__ and we'll have a look at it.

Cuando creas un issue. por favor asegúrate de:

* Incluir information suficiente para que otra persona pueda entender el problema.

Describe exactamente lo que estabas haciendo o tratando de hacer, y exactamente que fue lo que falló.
Si estabas siguiendo un tutorial en linea, suministra el enlace.

* Usa títulos descriptivos. Mal: "rviz no funciona". Bien: "Rviz dejo de funcionar luego de la ultima actualización del sistema"
* Incluye información exacta sobre la plataforma, version, software, y entorno de desarrollo relevante al problema. Esto incluye como instalaste los paquetes (a partir de binaries or desde la fuente). Ademas que proveedor de DDS estas (si lo sabes).
* Para cualquier advertencia o error. Copia y pega los mensajes directamente desde la terminal. Por favor no re-escribas los mensajes, ni incluyas capturas de pantalla.
* En caso de que sea un error, considera proporcionar un `ejemplo corto SSCCE <http://sscce.org/>`__.
* Cuando estés discutiendo algún problema de compilación o instalación, Por favor suministra la version del compilador.

También es adecuado incluir tus:

* Variables de entorno de ROS (env | grep ROS)
* Backtraces
* Archivos de configuración
* Modelo de tarjeta gráfica y versiones del driver
* Ogre.log para rviz, si es posible (ejecuta rviz con rviz -l)
* Archivos bag files y códigos de ejemplo que reproduzcan el problema
* Gifs o videos que demuestren el problema


Pull requests
--------------

Cuando te sientas lo suficientemente cómodo para sugerir un cambio especifico en el código, puedes solicitar directamente un pull request.
Los pull requests son bienvenidos en cualquier `repositorio de ros2 <https://github.com/ros2>`__.
Consulta la pagina de :doc:`Contribución <The-ROS2-Project/Contributing>` para mas detalles sobre como contribuir en el código.

.. _Using ROS Discourse:

Discusión
----------

Para iniciar una discusion con otro miembro de la comunidad de ROS2, visita el foro oficial `ROS Discourse <https://discourse.ros.org/>`__.
EL contenido de la discusion debe ser de alto nivel;
este no es el lugar para realizar preguntas sobre codigo. Sin embargo, si es apropiado para iniciar una discusion sobre las mejores practicas o mejora de los estandares.

Discusiones sobre el desarrollo de ROS 2 ocurren en la sesion de `“Generacion futura de ROS” Discourse category <https://discourse.ros.org/c/ng-ros>`__.
Participar en estas discusiones es una forma importante de opinar sobre cómo funcionarán e implementarán las diferentes funciones de ROS 2.

La comunidad diversa detrás del ecosistema ROS es uno de sus mayores fuertes.
Alentamos a todos los miembros de la comunidad de ROS a participar en estas discusiones de diseño para que podamos aprovechar la experiencia de los miembros de la comunidad y tener en cuenta los variados casos de uso de ROS.

Etiqueta
----------

Asume que 'buena fe': Es fácil de malinterpretar en el significado o en el tono de los comentarios en internet.
Asumir que la buena fe otorga el beneficio de la duda a quienes intentan ayudarlo, evitando: insultar a los miembros de la comunidad bien intencionados y envenenar el estado de ánimo.
Asumir 'buena fe' al responder casi siempre funciona, incluso si la respuesta original no fue de hecho de buena fe.

Por favor, no envíe su pregunta más de una vez: La pregunta fue vista.
Si no recibiste una respuesta, es probable que nadie haya tenido tiempo de responderte.
Alternativamente, podría ser que nadie sepa la respuesta.
En cualquier caso, enviarlo de nuevo es una mala practica y es comparable a gritar.
Intenta elegir el foro que creas que encaja mejor y pregunta allí. Si se te remiten a un nuevo foro, proporciona un enlace a la discusión anterior.

En https://answers.ros.org puedes editar tu pregunta para proporcionar más detalles.
Cuantos más detalles incluyas en tu pregunta, más fácil será para otros ayudarte a encontrar su solución.

Se considera de mala educación enumerar sus fechas límite personales; los miembros de la comunidad que responden preguntas también las tienen.

No ruegues por ayuda.
Si hay alguien dispuesto y capaz de ayudarte con tu problema, generalmente obtendrás una respuesta.
Pedir respuestas más rápidas en la mayoría de casos tendrá un efecto negativo.

No agregue contenido no relacionado a las publicaciones.
El contenido de las publicaciones debe centrarse en el tema en cuestión y no incluir contenido no relacionado.
El contenido,enlaces y las imágenes no relacionadas con el tema se consideran spam.

Para publicar contenido comercial, te puede interesar `esta discusión <https://discourse.ros.org/t/sponsorship-notation-in-posts-on-ros-org/2078>`_.

Minimice referencias a contenido con muros de pago detrás.
El contenido publicado en `ROS Discourse <https://discourse.ros.org/>`__ y `ROS Answers <https://answers.ros.org/>`__ se aconseja que sea gratis y abierto a todos los usuarios.
Siempre que sea posible, las fuentes primarias deben ser gratuitas y abiertas, y el contenido de pago debe desempeñar un papel de apoyo.

Se deben evitar las publicaciones de un solo enlace.
En términos generales, publicar una respuesta de un solo enlace es menos útil y puede confundirse fácilmente con spam.
Además, los enlaces pueden dañarse con el tiempo o ser reemplazados.
Parafrasear el contenido de un enlace junto con cierta información contextual y atribución suele ser mucho más útil.

Contacto privado
-----------------

Si deseas comunicarte con nosotros de forma privada (por ejemplo, si tu pregunta contiene información confidencial para tu organización o proyecto, o si se trata de un problema de seguridad), puedes enviarnos un correo electrónico directamente a ros@osrfoundation.org.
