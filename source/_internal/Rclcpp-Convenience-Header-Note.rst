.. note::

   ``rclcpp/rclcpp.hpp`` is a *convenience* header.
   It pulls in the whole ``rclcpp`` API at once — nodes, publishers, subscriptions, services, timers, parameters, executors, rates, wait sets, and more — so every translation unit that includes it is compiled against features it never uses.

   Outside of the tutorials, prefer including only the headers for the specific API calls you actually use.
   For example, ``rclcpp::Node`` is declared in ``rclcpp/node.hpp``, ``rclcpp::spin`` in ``rclcpp/executors.hpp``, and ``rclcpp::init`` and ``rclcpp::shutdown`` in ``rclcpp/utilities.hpp``.
   The saving is largest in translation units that never create or spin a node — headers, plugins, and helper libraries that only need something small like ``rclcpp/qos.hpp`` or ``rclcpp/time.hpp`` — because ``rclcpp/node.hpp`` and ``rclcpp/executors.hpp`` are large in their own right.
   ``rclcpp/rclcpp.hpp`` is little more than a list of these headers, so it is a good place to start when working out which ones you need.
