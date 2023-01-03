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
