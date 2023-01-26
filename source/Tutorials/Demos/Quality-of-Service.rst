.. redirect-from::

    Quality-of-Service
    Tutorials/Quality-of-Service

Uso de configuraciones de calidad de servicio para redes con pérdidas
=====================================================================

.. contents:: Tabla de contenidos
   :depth: 2
   :local:

Historial
---------

Lee la página de documentación `sobre la configuración de QoS <../../Concepts/About-Quality-of-Service-Settings>` para obtener información general sobre el soporte disponible en ROS 2.

En esta demostración, generaremos un nodo que publica una imagen de cámara y otro que se suscribe a la imagen y la muestra en la pantalla.
Luego simularemos una conexión de red con pérdida entre ellos y mostraremos cómo las diferentes configuraciones de calidad de servicio manejan el enlace defectuoso.


Requisitos precios
------------------

Este tutorial asume que tienes una :doc:`instalación de ROS 2 <../../Installation>` y OpenCV.
Consulta la documentación de `OpenCV <http://docs.opencv.org/doc/tutorials/introduction/table_of_content_introduction/table_of_content_introduction.html#table-of-content-introduction>`__ para conocer las instrucciones de instalación.
También necesitarás el paquete ROS ``image_tools``.

.. tabs::

   .. group-tab:: Linux Binaries

      .. code-block:: bash

        sudo apt-get install ros-{DISTRO}-image-tools

   .. group-tab:: From Source

      .. code-block:: bash

        # Clona y compila el repositorio de demos usando la rama que coincida con tu instalación
        git clone https://github.com/ros2/demos.git -b {REPOS_FILE_BRANCH}


Ejecución de la demo
--------------------

Antes de ejecutar la demostración, asegurate de tener una cámara web en funcionamiento conectada a su computadora.

Una vez que hayas instalado ROS 2, obtén tu archivo de instalación:

.. tabs::

  .. group-tab:: Linux

    .. code-block:: bash

       . <path to ROS 2 install space>/setup.bash

  .. group-tab:: macOS

    .. code-block:: bash

       . <path to ROS 2 install space>/setup.bash

  .. group-tab:: Windows

    .. code-block:: bash

       call <path to ROS 2 install space>/local_setup.bat

Entonces ejecuta:

.. code-block:: bash

   ros2 run image_tools showimage

No pasará nada todavía.
``showimage`` es un nodo suscriptor que está esperando un editor en el tema ``image``.

Nota: tienes que cerrar el proceso ``showimage`` con ``Ctrl-C`` más tarde.
No puedes simplemente cerrar la ventana.

En una terminal separada, obtén el archivo de instalación y ejecuta el nodo publicador:

.. code-block:: bash

   ros2 run image_tools cam2image

Esto publicará una imagen de su cámara web.
En caso de que no tengas una cámara conectada a su ordenador, hay una opción de línea de comandos que publica imágenes predefinidas.


.. code-block:: bash

   ros2 run image_tools cam2image --ros-args -p burger_mode:=True


En esta ventana, verás la salida del terminal:

.. code-block:: bash

   Publishing image #1
   Publishing image #2
   Publishing image #3
   ...

Aparecerá una ventana con el título "vista" que muestra la transmisión de su cámara.
En la primera ventana, verás el resultado del suscriptor:

.. code-block:: bash

   Received image #1
   Received image #2
   Received image #3
   ...

.. note::

   Usuarios de macOS: si estos ejemplos no funcionan o si recibes un error como ``ddsi_conn_write fail -1``, deberás aumentar el tamaño del paquete UDP en todo el sistema:

   .. code-block:: bash

      $ sudo sysctl -w net.inet.udp.recvspace=209715
      $ sudo sysctl -w net.inet.udp.maxdgram=65500

   Estos cambios no persistirán al reiniciar. Si deseas que los cambios persistan, agrega estas líneas a ``/etc/sysctl.conf`` (crea el archivo si aún no existe):

   .. code-block:: bash

      net.inet.udp.recvspace=209715
      net.inet.udp.maxdgram=65500

Opciones de la línea de comandos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

En uno de tus terminales, agrega un indicador -h al comando original:


.. code-block:: bash

   ros2 run image_tools showimage -h



Agregar tráfico de red
^^^^^^^^^^^^^^^^^^^^^^

.. warning::

  Esta sección de la demo no funcionará en Connext DDS de RTI.
  Cuando se ejecutan varios nodos en el mismo host, la implementación de RTI Connext DDS usa memoria compartida junto con la interfaz de bucle invertido.
  La degradación del rendimiento de la interfaz de bucle invertido no afectará la memoria compartida, por lo que el tráfico entre los dos nodos no se verá afectado.

.. note::

   La siguiente sección es específica de Linux.

   Sin embargo, para macOS y Windows puedes lograr un efecto similar con las utilidades "Network Link Conditioner" (parte del conjunto de herramientas xcode) y "Clumsy" (http://jagt.github.io/clumsy/index.html), respectivamente, pero no se tratarán en este tutorial.

Vamos a utilizar la utilidad de control de tráfico de red de Linux, ``tc`` (http://linux.die.net/man/8/tc).

.. code-block:: bash

   sudo tc qdisc add dev lo root netem loss 5%

Este conjuro mágico simulará una pérdida de paquetes del 5% en el dispositivo de bucle invertido local.
Si usas una resolución más alta de las imágenes (por ejemplo, ``--ros-args -p width:=640 -p height:=480``), es posible que desees probar una tasa de pérdida de paquetes más baja (por ejemplo, ``1%` `).

A continuación, iniciamos ``cam2image`` y ``showimage``, y pronto notaremos que ambos programas parecen haber disminuido la velocidad a la que se transmiten las imágenes.
Esto se debe al comportamiento de la configuración de QoS predeterminada.
Hacer cumplir la confiabilidad en un canal con pérdida significa que el editor (en este caso, ``cam2image``) reenviará los paquetes de red hasta que reciba el reconocimiento del consumidor (es decir, ``showimage``).

Intentaremos ahora ejecutar ambos programas, pero con configuraciones más adecuadas.
En primer lugar, usaremos la opción ``-p reliability:=best_effort`` para habilitar la comunicación de best effort.
El editor ahora solo intentará entregar los paquetes de red y no esperará el reconocimiento del consumidor.
Vemos ahora que algunos de los cuadros en el lado de ``showimage`` se eliminaron, por lo que los números de cuadro en el shell que ejecuta ``showimage`` ya no serán consecutivos:


.. image:: https://raw.githubusercontent.com/ros2/demos/{REPOS_FILE_BRANCH}/image_tools/doc/qos-best-effort.png
   :target: https://raw.githubusercontent.com/ros2/demos/{REPOS_FILE_BRANCH}/image_tools/doc/qos-best-effort.png
   :alt: Best effort image transfer


Cuando hayas terminado, recuerda eliminar la disciplina de cola:

.. code-block:: bash

   sudo tc qdisc delete dev lo root netem loss 5%
