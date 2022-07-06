NVIDIA HUMBLE ROS 2 Projects
=====================

NVIDIA provides packages for the development of AI applications for robotics.

ISAAC ROS Projects
------------
* `Isaac ROS Nitros <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros>`__: Isaac Transport for ROS package for hardware-acceleration friendly movement of messages.
* `Isaac ROS DNN Stereo Disparity <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_disparity>`__: DNN Stereo Disparity includes packages for predicting disparity of stereo input.
* `Isaac ROS Proximity Segmentation <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_proximity_segmentation>`__: Hardware-accelerated packages for proximity segmentation.
* `Isaac ROS Nvblox <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox>`__ : Hardware-accelerated 3D scene reconstruction and Nav2 local costmap provider using nvblox.
* `Isaac ROS Object Detection <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection>`__ : Deep learning model support for object detection including DetectNet.
* `Isaac ROS DNN Inference <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference>`__ : This repository provides two NVIDIA GPU-accelerated ROS 2 nodes that perform deep learning inference using custom models. One node uses the TensorRT SDK, while the other uses the Triton SDK.
* `Isaac ROS Visual SLAM <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam>`__ : This repository provides a ROS 2 package that estimates stereo visual inertial odometry using the Isaac Elbrus GPU-accelerated library.
* `Isaac ROS Argus Camera <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_argus_camera>`__ : This repository provides monocular and stereo nodes that enable ROS developers to use cameras connected to Jetson platforms over a CSI interface.
* `Isaac ROS image_pipeline <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline>`__ : This metapackage offers similar functionality as the standard, CPU-based image_pipeline metapackage, but does so by leveraging the Jetson platform's specialized computer vision hardware.
* `Isaac ROS Common <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common>`__ : Isaac ROS common utilities for use in conjunction with the Isaac ROS suite of packages.
* `Isaac ROS AprilTags <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag>`__ : ROS 2 node uses the NVIDIA GPU-accelerated AprilTags library to detect AprilTags in images and publish their poses, ids, and additional metadata.

Additional Projects
-------------------
* `ROS and ROS 2 DockerFiles <https://github.com/dusty-nv/jetson-containers>`__: Dockerfiles for ROS 2 based on l4t which all you to build your own Docker image.
* `ROS / ROS 2 Packages for Accelerated Deep Learning Nodes <https://github.com/dusty-nv/ros_deep_learning>`__: Deep learning image recognition, object detection, and semantic segmentation inference nodes and camera/video streaming nodes for ROS/ROS 2 using the `jetson-inference <https://github.com/dusty-nv/jetson-inference>`__ library and `NVIDIA Hello AI World tutorial <https://developer.nvidia.com/embedded/twodaystoademo>`__.
Simulation Projects
-------------------
* `Isaac Sim Nav2 <https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_ros2_navigation.html>`__ : In this ROS 2 sample, we are demonstrating Omniverse Isaac Sim integrated with the ROS 2 Nav2 project.
* `Isaac Sim Multiple Robot ROS 2 Navigation <https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_ros2_multi_navigation.html>`__ : In this ROS 2 sample, we are demonstrating Omniverse Isaac Sim integrated with the ROS 2 Nav2 stack to perform simultaneous multiple robot navigation.