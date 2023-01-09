 rolling
Proyectos de ROS 2 por NVIDIA
=============================

NVIDIA Jetson trabaja en el desarrollo de paquetes de ROS2 con el find e facilitar el desarrollo de inteligencia artificial para la robótica.


ROS Projects
------------
* `Isaac ROS Nvblox <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox>`__ : Reconstruccion 3D de escenas y costmap locales para Nav2 usando nvblox.
* `Isaac ROS Detección de objetos <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection>`__ : Modelos de detección de objeto incluyendo DetectNet.
* `Isaac ROS Inferencia DNN <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference>`__ : Este repositorio contiene nodos de ROS 2 que realizan inferencias usando diferentes modelos de inteligencia artificial. Uno de los nodos usa TensorRT SDK, mientras que el otro usa Triton SDK.
* `Isaac ROS SLAM Visual <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam>`__ : Este repositorio contiene un paquete de ROS 2 que estima la odometría visual inercial usando la librería Isaac Elbrus GPU.
* `Isaac ROS Cámara Argus <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_argus_camera>`__ : Este repositorio permite usar cámaras conectadas a plataformas Jetson mediante una interfaz CSI.
* `Isaac ROS image_pipeline <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline>`__ : Este paquete ofrece funcionalidades estándar de vision por computadora enfocado a plataformas Jetson.
* `Isaac ROS Common <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common>`__ : Utilidades comunes para usar junto con Isaac ROS.
* `Isaac ROS AprilTags <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag>`__ : Nodo de ROS2 que usa la librería AprilTags en imágenes, publicando sus posiciones, ids e información adicional.
* `ROS y ROS 2 Docker Images <https://github.com/NVIDIA-AI-IOT/ros2_jetson/tree/main/docker>`__ : Imágenes Docker para ROS2
* `ROS y ROS 2 DockerFiles <https://github.com/dusty-nv/jetson-containers>`__: Dockerfiles para ROS 2 based.
* `Paquetes ROS 2 para PyTorch y TensorRT <https://github.com/NVIDIA-AI-IOT/ros2_torch_trt>`__: Paquetes ROS 2 para clasificación y detección de objetos usando PyTorch y NVIDIA TensorRT.
* `Paquetes ROS / ROS 2 para Nodos con Deep Learning <https://github.com/dusty-nv/ros_deep_learning>`__: Reconocimiento de imágenes, detección de objetos y segmentación semántica usando `jetson-inference <https://github.com/dusty-nv/jetson-inference>`__ y `NVIDIA Hello AI World tutorial <https://developer.nvidia.com/embedded/twodaystoademo>`__.
* `Paquete de ROS 2 Package para estimar poses humanas <https://github.com/NVIDIA-AI-IOT/ros2_trt_pose>`__: Paquete para estimar poses humanas
* `Paquetes de ROS 2 para estimar poses de manos humanas y clasificacion de gestos <https://github.com/NVIDIA-AI-IOT/ros2_trt_pose_hand>`__: Paquete para estimar poses de manos humanas en tiempo real asi como clasificación de gestos usando TensorRT.
* `Paquetes de ROS2 para estimación de profundidad monocular <https://github.com/NVIDIA-AI-IOT/ros2_torch2trt_examples>`__: Paquete de ROS2 para usar la librería.
* `Paquetes de ROS 2 para estadísticas de Jetson <https://github.com/NVIDIA-AI-IOT/ros2_jetson_stats>`__: Paquete para monitorear y controlar tu NVIDIA Jetson [Xavier NX, Nano, AGX Xavier, TX1, TX2].
* `Paquetes de ROS 2 para DeepStream SDK <https://github.com/NVIDIA-AI-IOT/ros2_deepstream>`__: Paquete para NVIDIA DeepStream SDK.

Proyectos de simulación
-----------------------
* `Isaac Sim Nav2 <https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_ros2_navigation.html>`__ : Demostraciones del simulador Isaac Sim junto con ROS 2 Nav2.
* `Isaac Sim Multiple Robot ROS 2 Navigation <https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_ros2_multi_navigation.html>`__ : Demostraciones del simulador Isaac Sim, para la navegación de multiples robots usando Nav2.

Referencias
-----------
Actualizaciones de Jetson ROS 2 se pueden encontrar `aca <https://nvidia-ai-iot.github.io/ros2_jetson/>`__.
=======
Proyectos NVIDIA ROS 2
======================

NVIDIA Jetson está trabajando para desarrollar paquetes ROS 2 para facilitar el desarrollo de aplicaciones de IA para robótica.


Proyectos ROS
-------------
* `Isaac ROS Nvblox <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox>`__ : Reconstrucción de escenas 3D acelerada por hardware y proveedor de mapa de costes local Nav2 usando nvblox.
* `Detección de objetos de ROS de Isaac <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection>`__: Compatibilidad con el modelo de aprendizaje profundo para la detección de objetos, incluido DetectNet.
* `Isaac ROS DNN Inference <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference>`__ : este repositorio proporciona dos nodos NVIDIA ROS 2 acelerados por GPU que realizan inferencias de aprendizaje profundo utilizando modelos personalizados. Un nodo usa el SDK de TensorRT, mientras que el otro usa el SDK de Triton.
* `Isaac ROS Visual SLAM <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam>`__ : este repositorio proporciona un paquete ROS 2 que estima la odometría inercial visual estéreo utilizando la biblioteca acelerada por GPU de Isaac Elbrus.
* `Isaac ROS Argus Camera <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_argus_camera>`__ : este repositorio proporciona nodos monoculares y estéreo que permiten a los desarrolladores de ROS usar cámaras conectadas a plataformas Jetson a través de una interfaz CSI.
* `Isaac ROS image_pipeline <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline>`__ : este metapaquete ofrece una funcionalidad similar al metapaquete image_pipeline estándar basado en CPU, pero lo hace aprovechando el hardware de la plataforma Jetson especializada en visión.
* `Isaac ROS Common <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common>`__ : Utilidades comunes de Isaac ROS para usar junto con la suite de paquetes Isaac ROS.
* `Isaac ROS AprilTags <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag>`__ : el nodo ROS 2 utiliza la biblioteca AprilTags acelerada por GPU de NVIDIA para detectar AprilTags en imágenes y publicar sus poses, ID y otros metadatos
* `Imágenes de Docker de ROS y ROS 2 <https://github.com/NVIDIA-AI-IOT/ros2_jetson/tree/main/docker>`__: imágenes de Docker para una fácil implementación en la plataforma NVIDIA Jetson, que consta de ROS 2, PyTorch y otras importantes bibliotecas de aprendizaje automático.
* `ROS y ROS 2 DockerFiles <https://github.com/dusty-nv/jetson-containers>`__: Dockerfiles para ROS 2 basados en l4t que le permiten crear su propia imagen de Docker.
* `Paquetes ROS 2 para PyTorch y TensorRT <https://github.com/NVIDIA-AI-IOT/ros2_torch_trt>`__: El paquete ROS 2 es para tareas de clasificación y detección de objetos usando PyTorch y NVIDIA TensorRT. Este tutorial es un buen punto de partida para la integración de IA con ROS 2 en NVIDIA Jetson.
* `Paquetes ROS / ROS 2 para nodos de aprendizaje profundo acelerado <https://github.com/dusty-nv/ros_deep_learning>`__: Reconocimiento de imágenes de aprendizaje profundo, detección de objetos y nodos de inferencia de segmentación semántica y nodos de transmisión de cámara/video para ROS/ROS 2 utilizando la biblioteca `jetson-inference <https://github.com/dusty-nv/jetson-inference>`__ y el tutorial `NVIDIA Hello AI World <https://developer.nvidia.com/embedded/ dosdíasparademostración>`__.
* `Paquete ROS 2 para la estimación de la pose humana <https://github.com/NVIDIA-AI-IOT/ros2_trt_pose>`__: un paquete ROS 2 para la estimación de la pose humana.
* `Paquete ROS 2 para estimación de posturas de manos y clasificación de gestos <https://github.com/NVIDIA-AI-IOT/ros2_trt_pose_hand>`__: un paquete ROS 2 para estimación de posturas de manos y clasificación de gestos en tiempo real mediante TensorRT.
* `Paquetes ROS 2 acelerados por GPU para estimación de profundidad monocular <https://github.com/NVIDIA-AI-IOT/ros2_torch2trt_examples>`__: paquete ROS 2 para ejemplos de torch2trtxb acelerados por GPU de NVIDIA, como estimación de profundidad monocular y detección de texto.
* `Paquete ROS 2 para estadísticas Jetson <https://github.com/NVIDIA-AI-IOT/ros2_jetson_stats>`__: Paquete ROS 2 para monitorear y controlar su NVIDIA Jetson [Xavier NX, Nano, AGX Xavier, TX1, TX2 ].
* `Paquetes ROS 2 para DeepStream SDK <https://github.com/NVIDIA-AI-IOT/ros2_deepstream>`__: Paquete ROS 2 para NVIDIA DeepStream SDK.

Proyectos de simulación
-----------------------
* `Isaac Sim Nav2 <https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_ros2_navigation.html>`__: En esta demo de ROS 2, mostramos Omniverse Isaac Sim integrado con el proyecto ROS 2 Nav2.
* `Isaac Sim Multiple Robot ROS 2 Navigation <https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_ros2_multi_navigation.html>`__ : En esta demo de ROS 2, mostramos Omniverse Isaac Sim integrado con ROS 2 Pila Nav2 para realizar la navegación simultánea de varios robots.

Referencias
-----------
Se pueden encontrar más actualizaciones sobre NVIDIA Jetson ROS 2 `aquí <https://nvidia-ai-iot.github.io/ros2_jetson/>`__.
 rolling
