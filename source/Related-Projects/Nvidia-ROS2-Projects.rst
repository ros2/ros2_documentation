NVIDIA ROS 2 Projects
=====================

NVIDIA Jetson is working towards developing ROS 2 packages to ease the development of AI applications for robotics.


Key Projects
------------

* `ROS2: Isaac ROS DNN Inference <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference>`__ : This repository provides two NVIDIA GPU-accelerated ROS2 nodes that perform deep learning inference using custom models. One node uses the TensorRT SDK, while the other uses the Triton SDK.
* `ROS2: Isaac ROS Visual Odometry <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_odometry>`__ : This repository provides a ROS2 package that estimates stereo visual inertial odometry using the Isaac Elbrus GPU-accelerated library.
* `ROS2: Isaac ROS Argus Camera <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_argus_camera>`__ : This repository provides monocular and stereo nodes that enable ROS developers to use cameras connected to Jetson platforms over a CSI interface.
* `ROS2: Isaac ROS image_pipeline <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline>`__ : This metapackage offers similar functionality as the standard, CPU-based image_pipeline metapackage, but does so by leveraging the Jetson platform's specialized computer vision hardware.
* `ROS2: Isaac ROS Common <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common>`__ : Isaac ROS common utilities for use in conjunction with the Isaac ROS suite of packages.
* `ROS2 Package for Accelerated AprilTags <https://github.com/NVIDIA-ISAAC-ROS/ros2-nvapriltags>`__ : ROS2 node uses the NVIDIA GPU-accelerated AprilTags library to detect AprilTags in images and publish their poses, ids, and additional metadata.
* `ROS and ROS 2 Docker Images <https://github.com/NVIDIA-AI-IOT/ros2_jetson/tree/main/docker>`__ : Docker images for easy deployment on the NVIDIA Jetson platform, consisting of ROS 2, PyTorch, and other important machine learning libraries.
* `ROS and ROS 2 DockerFiles <https://github.com/dusty-nv/jetson-containers>`__: Dockerfiles for ROS 2 based on l4t which all you to build your own Docker image.
* `ROS 2 Packages for PyTorch and TensorRT <https://github.com/NVIDIA-AI-IOT/ros2_torch_trt>`__: ROS 2 packageis for classification and object detection tasks using PyTorch and NVIDIA TensorRT. This tutorial is a good starting point AI integration with ROS 2 on NVIDIA Jetson.
* `ROS / ROS 2 Packages for Accelerated Deep Learning Nodes <https://github.com/dusty-nv/ros_deep_learning>`__: Deep learning image recognition, object detection, and semantic segmentation inference nodes and camera/video streaming nodes for ROS/ROS 2 using the `jetson-inference <https://github.com/dusty-nv/jetson-inference>`__ library and `NVIDIA Hello AI World tutorial <https://developer.nvidia.com/embedded/twodaystoademo>`__.
* `ROS 2 Package for Human Pose Estimation <https://github.com/NVIDIA-AI-IOT/ros2_trt_pose>`__: A ROS 2 package for human pose estimation.
* `ROS 2 Package for Hand Pose Estimation and Gesture Classification <https://github.com/NVIDIA-AI-IOT/ros2_trt_pose_hand>`__: A ROS 2 package for real-time hand pose estimation and gesture classification using TensorRT.
* `GPU accelerated ROS 2 Packages for Monocular Depth Estimation <https://github.com/NVIDIA-AI-IOT/ros2_torch2trt_examples>`__: ROS 2 package for NVIDIA GPU-accelerated torch2trtxb examples such as monocular depth estimation and text detection.
* `ROS 2 Package for Jetson stats <https://github.com/NVIDIA-AI-IOT/ros2_jetson_stats>`__: ROS 2 package for monitoring and controlling your NVIDIA Jetson [Xavier NX, Nano, AGX Xavier, TX1, TX2].
* `ROS 2 Packages for DeepStream SDK <https://github.com/NVIDIA-AI-IOT/ros2_deepstream>`__: ROS 2 package for NVIDIA DeepStream SDK.

References
----------

More updates on NVIDIA Jetson ROS 2 can be found `here <https://nvidia-ai-iot.github.io/ros2_jetson/>`__.
