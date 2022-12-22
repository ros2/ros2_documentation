.. redirect-from::

    Tutorials/Content-Filtering-Subscription

Creación de un suscriptor de filtrado de contenido
=========================================

**Objetivo:** Crear un suscriptor de filtrado de contenido.

**Nivel del Tutorial:** Avanzado

**Tiempo:** 15 minutos

.. contents:: Tabla de contenidos
   :depth: 1
   :local:

Descripción
--------

Las aplicaciones de ROS 2 generalmente consisten en topics para transmitir datos de los publicadores a las suscriptores.
Básicamente, los suscriptores reciben todos los datos publicados de los publicadores por el topic.
A veces, una suscripción puede estar interesada solo en un subconjunto de los datos que envían los publicadores.
Un suscriptor de filtrado de contenido permite recibir solo los datos de interés para la aplicación.

En esta demo, destacaremos cómo crear un suscriptor que filtra contenido y cómo funcionan.

Soporte RMW
-----------

Los suscriptores de filtrado de contenido requieren compatibilidad con la implementación de RMW.

.. list-table::  Estado de soporte de suscriptor de filtrado de contenido
   :widths: 25 25

   * - rmw_fastrtps
     - soportado
   * - rmw_connextdds
     - soportado
   * - rmw_cyclonedds
     - no soportado

Actualmente, todas las implementaciones de RMW que admiten suscripciones de filtrado de contenido están basadas en `DDS <https://www.omg.org/omg-dds-portal/>`__.
Eso significa que las expresiones y parámetros de filtrado compatibles también dependen de `DDS <https://www.omg.org/omg-dds-portal/>`__, puede consultar la especificación `DDS <https://www. omg.org/spec/DDS/1.4/PDF>`__ ``Annex B - Syntax for Queries and Filters`` para obtener detalles.

Instalación de la demo
-------------------

Consulte las :doc:`installation instructions <../../Installation>` para obtener detalles sobre la instalación de ROS 2.

Si ha instalado ROS 2 desde paquetes, asegúrese de tener ``ros-{DISTRO}-demo-nodes-cpp`` instalado.
Si descargó el archivo o creó ROS 2 desde fuentes, ya será parte de la instalación.

Demostración de filtrado de temperatura
--------------------------

Esta demostración muestra cómo se puede usar un suscriptor de filtrado de contenido para recibir solo valores de temperatura que están fuera del rango de temperatura aceptable, detectando emergencias.
El suscriptor de filtrado de contenido filtra los datos de temperatura poco interesantes, de modo que no se emite el callback del suscriptor

ContentFilteringPublisher:

https://github.com/ros2/demos/blob/{REPOS_FILE_BRANCH}/demo_nodes_cpp/src/topics/content_filtering_publisher.cpp

.. code-block:: c++

    #include <chrono>
    #include <cstdio>
    #include <memory>
    #include <utility>

    #include "rclcpp/rclcpp.hpp"
    #include "rclcpp_components/register_node_macro.hpp"

    #include "std_msgs/msg/float32.hpp"

    #include "demo_nodes_cpp/visibility_control.h"

    using namespace std::chrono_literals;

    namespace demo_nodes_cpp
    {
    // Los datos de temperatura simulados comienzan desde -100,0 y terminan en 150,0 con un tamaño de paso de 10,0
    constexpr std::array<float, 3> TEMPERATURE_SETTING {-100.0f, 150.0f, 10.0f};

    // Cree una clase ContentFilteringPublisher que cree subclases de la clase base genérica rclcpp::Node.
    // La función principal a continuación creará una instancia de la clase como un nodo ROS.
    class ContentFilteringPublisher : public rclcpp::Node
    {
    public:
      DEMO_NODES_CPP_PUBLIC
      explicit ContentFilteringPublisher(const rclcpp::NodeOptions & options)
      : Node("content_filtering_publisher", options)
      {
        // Cree una función para cuándo se deben enviar los mensajes.
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        auto publish_message =
          [this]() -> void
          {
            msg_ = std::make_unique<std_msgs::msg::Float32>();
            msg_->data = temperature_;
            temperature_ += TEMPERATURE_SETTING[2];
            if (temperature_ > TEMPERATURE_SETTING[1]) {
              temperature_ = TEMPERATURE_SETTING[0];
            }
            RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", msg_->data);
            // Poner el mensaje en una cola para que lo procese el middleware.
            // Esta llamada es no bloqueante.
            pub_->publish(std::move(msg_));
          };
        // Cree un publicador con un perfil de calidad de servicio personalizado.
        // Se sugiere una inicialización uniforme para que pueda cambiarse trivialmente a
        // rclcpp::KeepAll{} si el usuario lo desea.
        // (rclcpp::KeepLast(7) -> rclcpp::KeepAll() falla al compilar)
        rclcpp::QoS qos(rclcpp::KeepLast{7});
        pub_ = this->create_publisher<std_msgs::msg::Float32>("temperature", qos);

        // Use un temporizador para programar la publicación periódica de mensajes.
        timer_ = this->create_wall_timer(1s, publish_message);
      }

    private:
      float temperature_ = TEMPERATURE_SETTING[0];
      std::unique_ptr<std_msgs::msg::Float32> msg_;
      rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
      rclcpp::TimerBase::SharedPtr timer_;
    };

    }  // namespace demo_nodes_cpp

El filtro de contenido se define en el lado de la suscripción, los publicadores no necesitan configurarse de ninguna manera especial para permitir el filtrado de contenido.
El nodo ``ContentFilteringPublisher`` publica datos de temperatura simulados desde -100,0 hasta 150,0 con un tamaño de paso de 10,0 cada segundo.

Podemos ejecutar la demo ejecutando ``ros2 run demo_nodes_cpp content_filtering_publisher`` (no olvide hacer source del install.bash del paquete):

.. code-block:: bash

    $ ros2 run demo_nodes_cpp content_filtering_publisher
    [INFO] [1651094594.822753479] [content_filtering_publisher]: Publishing: '-100.000000'
    [INFO] [1651094595.822723857] [content_filtering_publisher]: Publishing: '-90.000000'
    [INFO] [1651094596.822752996] [content_filtering_publisher]: Publishing: '-80.000000'
    [INFO] [1651094597.822752475] [content_filtering_publisher]: Publishing: '-70.000000'
    [INFO] [1651094598.822721485] [content_filtering_publisher]: Publishing: '-60.000000'
    [INFO] [1651094599.822696188] [content_filtering_publisher]: Publishing: '-50.000000'
    [INFO] [1651094600.822699217] [content_filtering_publisher]: Publishing: '-40.000000'
    [INFO] [1651094601.822744113] [content_filtering_publisher]: Publishing: '-30.000000'
    [INFO] [1651094602.822694805] [content_filtering_publisher]: Publishing: '-20.000000'
    [INFO] [1651094603.822735805] [content_filtering_publisher]: Publishing: '-10.000000'
    [INFO] [1651094604.822722094] [content_filtering_publisher]: Publishing: '0.000000'
    [INFO] [1651094605.822699960] [content_filtering_publisher]: Publishing: '10.000000'
    [INFO] [1651094606.822748946] [content_filtering_publisher]: Publishing: '20.000000'
    [INFO] [1651094607.822694017] [content_filtering_publisher]: Publishing: '30.000000'
    [INFO] [1651094608.822708798] [content_filtering_publisher]: Publishing: '40.000000'
    [INFO] [1651094609.822692417] [content_filtering_publisher]: Publishing: '50.000000'
    [INFO] [1651094610.822696426] [content_filtering_publisher]: Publishing: '60.000000'
    [INFO] [1651094611.822751913] [content_filtering_publisher]: Publishing: '70.000000'
    [INFO] [1651094612.822692231] [content_filtering_publisher]: Publishing: '80.000000'
    [INFO] [1651094613.822745549] [content_filtering_publisher]: Publishing: '90.000000'
    [INFO] [1651094614.822701982] [content_filtering_publisher]: Publishing: '100.000000'
    [INFO] [1651094615.822691465] [content_filtering_publisher]: Publishing: '110.000000'
    [INFO] [1651094616.822649070] [content_filtering_publisher]: Publishing: '120.000000'
    [INFO] [1651094617.822693616] [content_filtering_publisher]: Publishing: '130.000000'
    [INFO] [1651094618.822691832] [content_filtering_publisher]: Publishing: '140.000000'
    [INFO] [1651094619.822688452] [content_filtering_publisher]: Publishing: '150.000000'
    [INFO] [1651094620.822645327] [content_filtering_publisher]: Publishing: '-100.000000'
    [INFO] [1651094621.822689219] [content_filtering_publisher]: Publishing: '-90.000000'
    [INFO] [1651094622.822694292] [content_filtering_publisher]: Publishing: '-80.000000'
    [...]

ContentFilteringSubscriber:

https://github.com/ros2/demos/blob/{REPOS_FILE_BRANCH}/demo_nodes_cpp/src/topics/content_filtering_subscriber.cpp

.. code-block:: c++

    #include "rclcpp/rclcpp.hpp"
    #include "rclcpp_components/register_node_macro.hpp"
    #include "rcpputils/join.hpp"

    #include "std_msgs/msg/float32.hpp"

    #include "demo_nodes_cpp/visibility_control.h"

    namespace demo_nodes_cpp
    {
    // Datos de temperatura de emergencia inferiores a -30 o superiores a 100
    constexpr std::array<float, 2> EMERGENCY_TEMPERATURE {-30.0f, 100.0f};

    // Cree una clase ContentFilteringSubscriber que cree subclases de la clase base genérica rclcpp::Node.
    // La función principal a continuación creará una instancia de la clase como un nodo ROS.
    class ContentFilteringSubscriber : public rclcpp::Node
    {
    public:
      DEMO_NODES_CPP_PUBLIC
      explicit ContentFilteringSubscriber(const rclcpp::NodeOptions & options)
      : Node("content_filtering_subscriber", options)
      {
        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        // Crear una función callback para cuando se reciban mensajes.
        auto callback =
          [this](const std_msgs::msg::Float32 & msg) -> void
          {
            if (msg.data < EMERGENCY_TEMPERATURE[0] || msg.data > EMERGENCY_TEMPERATURE[1]) {
              RCLCPP_INFO(
                this->get_logger(),
                "I receive an emergency temperature data: [%f]", msg.data);
            } else {
              RCLCPP_INFO(this->get_logger(), "I receive a temperature data: [%f]", msg.data);
            }
          };

        // Inicialice una suscripción con un filtro de contenido para recibir datos de temperatura de emergencia que
        // son menores que -30 o mayores que 100.
        rclcpp::SubscriptionOptions sub_options;
        sub_options.content_filter_options.filter_expression = "data < %0 OR data > %1";
        sub_options.content_filter_options.expression_parameters = {
          std::to_string(EMERGENCY_TEMPERATURE[0]),
          std::to_string(EMERGENCY_TEMPERATURE[1])
        };

        sub_ = create_subscription<std_msgs::msg::Float32>("temperature", 10, callback, sub_options);

        if (!sub_->is_cft_enabled()) {
          RCLCPP_WARN(
            this->get_logger(), "Content filter is not enabled since it's not supported");
        } else {
          RCLCPP_INFO(
            this->get_logger(),
            "subscribed to topic \"%s\" with content filter options \"%s, {%s}\"",
            sub_->get_topic_name(),
            sub_options.content_filter_options.filter_expression.c_str(),
            rcpputils::join(sub_options.content_filter_options.expression_parameters, ", ").c_str());
        }
      }

    private:
      rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
    };

    }  // namespace demo_nodes_cpp

Para habilitar el filtrado de contenido, las aplicaciones pueden configurar la expresión de filtrado y los parámetros de expresión en ``SubscriptionOptions``.
La aplicación también puede verificar si el filtrado de contenido está habilitado en la suscripción.

En esta demo, el nodo ``ContentFilteringSubscriber`` crea una suscripción de filtrado de contenido que recibe un mensaje solo si el valor de la temperatura es inferior a -30,0 o superior a 100,0.

Como se comentó anteriormente, el soporte de suscripción de filtrado de contenido depende de la implementación de RMW.
Las aplicaciones pueden usar el método ``is_cft_enabled`` para verificar si el filtrado de contenido está realmente habilitado en la suscripción.

Para probar la suscripción de filtrado de contenido, ejecutémoslo:

.. code-block:: bash

    $ ros2 run demo_nodes_cpp content_filtering_subscriber
    [INFO] [1651094590.682660703] [content_filtering_subscriber]: subscribed to topic "/temperature" with content filter options "data < %0 OR data > %1, {-30.000000, 100.000000}"
    [INFO] [1651094594.823805294] [content_filtering_subscriber]: I receive an emergency temperature data: [-100.000000]
    [INFO] [1651094595.823419993] [content_filtering_subscriber]: I receive an emergency temperature data: [-90.000000]
    [INFO] [1651094596.823410859] [content_filtering_subscriber]: I receive an emergency temperature data: [-80.000000]
    [INFO] [1651094597.823350377] [content_filtering_subscriber]: I receive an emergency temperature data: [-70.000000]
    [INFO] [1651094598.823282657] [content_filtering_subscriber]: I receive an emergency temperature data: [-60.000000]
    [INFO] [1651094599.823297857] [content_filtering_subscriber]: I receive an emergency temperature data: [-50.000000]
    [INFO] [1651094600.823355597] [content_filtering_subscriber]: I receive an emergency temperature data: [-40.000000]
    [INFO] [1651094615.823315377] [content_filtering_subscriber]: I receive an emergency temperature data: [110.000000]
    [INFO] [1651094616.823258458] [content_filtering_subscriber]: I receive an emergency temperature data: [120.000000]
    [INFO] [1651094617.823323525] [content_filtering_subscriber]: I receive an emergency temperature data: [130.000000]
    [INFO] [1651094618.823315527] [content_filtering_subscriber]: I receive an emergency temperature data: [140.000000]
    [INFO] [1651094619.823331424] [content_filtering_subscriber]: I receive an emergency temperature data: [150.000000]
    [INFO] [1651094620.823271748] [content_filtering_subscriber]: I receive an emergency temperature data: [-100.000000]
    [INFO] [1651094621.823343550] [content_filtering_subscriber]: I receive an emergency temperature data: [-90.000000]
    [INFO] [1651094622.823286326] [content_filtering_subscriber]: I receive an emergency temperature data: [-80.000000]
    [INFO] [1651094623.823371031] [content_filtering_subscriber]: I receive an emergency temperature data: [-70.000000]
    [INFO] [1651094624.823333112] [content_filtering_subscriber]: I receive an emergency temperature data: [-60.000000]
    [INFO] [1651094625.823266469] [content_filtering_subscriber]: I receive an emergency temperature data: [-50.000000]
    [INFO] [1651094626.823284093] [content_filtering_subscriber]: I receive an emergency temperature data: [-40.000000]

Debería ver un mensaje que muestre las opciones de filtrado de contenido utilizadas y los registros de cada mensaje recibido solo si el valor de la temperatura es inferior a -30,0 o superior a 100,0.

Si la implementación de RMW no admite el filtrado de contenido, la suscripción aún se creará sin el filtrado de contenido habilitado.
Podemos intentarlo ejecutando ``RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run demo_nodes_cpp content_filtering_publisher``.

.. code-block:: bash

    $ RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run demo_nodes_cpp content_filtering_subscriber
    [WARN] [1651096637.893842072] [content_filtering_subscriber]: Content filter is not enabled since it is not supported
    [INFO] [1651096641.246043703] [content_filtering_subscriber]: I receive an emergency temperature data: [-100.000000]
    [INFO] [1651096642.245833527] [content_filtering_subscriber]: I receive an emergency temperature data: [-90.000000]
    [INFO] [1651096643.245743471] [content_filtering_subscriber]: I receive an emergency temperature data: [-80.000000]
    [INFO] [1651096644.245833932] [content_filtering_subscriber]: I receive an emergency temperature data: [-70.000000]
    [INFO] [1651096645.245916679] [content_filtering_subscriber]: I receive an emergency temperature data: [-60.000000]
    [INFO] [1651096646.245861895] [content_filtering_subscriber]: I receive an emergency temperature data: [-50.000000]
    [INFO] [1651096647.245946352] [content_filtering_subscriber]: I receive an emergency temperature data: [-40.000000]
    [INFO] [1651096648.245934569] [content_filtering_subscriber]: I receive a temperature data: [-30.000000]
    [INFO] [1651096649.245877906] [content_filtering_subscriber]: I receive a temperature data: [-20.000000]
    [INFO] [1651096650.245939068] [content_filtering_subscriber]: I receive a temperature data: [-10.000000]
    [INFO] [1651096651.245911450] [content_filtering_subscriber]: I receive a temperature data: [0.000000]
    [INFO] [1651096652.245879830] [content_filtering_subscriber]: I receive a temperature data: [10.000000]
    [INFO] [1651096653.245858329] [content_filtering_subscriber]: I receive a temperature data: [20.000000]
    [INFO] [1651096654.245916370] [content_filtering_subscriber]: I receive a temperature data: [30.000000]
    [INFO] [1651096655.245933741] [content_filtering_subscriber]: I receive a temperature data: [40.000000]
    [INFO] [1651096656.245833975] [content_filtering_subscriber]: I receive a temperature data: [50.000000]
    [INFO] [1651096657.245971483] [content_filtering_subscriber]: I receive a temperature data: [60.000000]

Puede ver el mensaje ``Content filter is not enabled`` porque la implementación de RMW subyacente no es compatible con la función, pero la demostración aún crea correctamente la suscripción normal para recibir todos los datos de temperatura.

Contenido relacionado
---------------

- `ejemplos de filtrado de contenido <https://github.com/ros2/examples/blob/{REPOS_FILE_BRANCH}/rclcpp/topics/minimal_subscriber/content_filtering.cpp>`__  que cubren todas las interfaces para la suscripción de filtrado de contenido.

- `diseño de filtrado de contenido PR <https://github.com/ros2/design/pull/282>`__
