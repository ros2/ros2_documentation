GPU buffer transport with ``rosidl::Buffer``
============================================

.. contents:: Table of Contents
   :depth: 2
   :local:

Background
----------

:doc:`rosidl::Buffer backends <../../Concepts/Intermediate/About-Buffer-Backends>`
let ROS 2 messages carry large payloads (images, tensors, ...) in
vendor-specific memory domains such as GPU memory, with the descriptor
round-trip handled transparently by the RMW.

This demo exercises the full pipeline using the community-maintained CUDA
and PyTorch backends, publishing ``sensor_msgs/msg/Image`` frames between
two processes either over a zero-copy CUDA path or over the traditional
CPU-serialised path, and comparing throughput at several resolutions.

The underlying demo is maintained in
`ros2/rosidl_buffer_backends_tutorials
<https://github.com/ros2/rosidl_buffer_backends_tutorials>`__; the package
name is ``robot_arm_demo``.

What the demo does
------------------

``robot_arm_demo`` renders an SDF-based pencil-sketch robot arm animation
entirely on the GPU via LibTorch tensor operations, publishes BGRA frames
as ``sensor_msgs/msg/Image``, and displays them in an SDL2/OpenGL window
with CUDA-GL interop. Two processes are involved:

#. ``renderer_node`` -- renders BGRA frames on the GPU using LibTorch
   operations and publishes them as ``sensor_msgs/msg/Image``.
#. ``display_node`` -- subscribes to the image topic, displays frames, and
   reports FPS.

Two transport modes are compared:

* **CUDA** -- the ``Image.data`` field is backed by the ``torch`` buffer
  backend which in turn sits on top of the ``cuda`` base backend.
  The bytes never leave GPU memory: the descriptor on the wire only
  carries a CUDA IPC handle.
* **CPU** -- the frame is rendered on the GPU, copied back to host memory
  with ``cudaMemcpy``, and then serialised through the RMW as a regular
  ``uint8[]``. No buffer backend is involved.

Both modes render on the GPU; the only difference is the transport path,
making this a clean comparison of zero-copy CUDA IPC versus traditional
CPU-serialised image transport.

Prerequisites
-------------

This demo is not part of the ROS 2 binary distribution; it lives in its own
repository and has GPU-specific dependencies. You need:

* A CUDA-capable GPU and the CUDA Toolkit (>= 11.8).
* SDL2, GLEW, OpenGL, X11 development packages.
* A ROS 2 Rolling source workspace. See the :doc:`Installation
  instructions <../../Installation>` for the canonical source-build flow.

The demo's ``libtorch_vendor`` package will download and install a
pre-built LibTorch distribution automatically at build time if one is not
already visible on the system.

Building
--------

Clone both repositories into your ROS 2 workspace's ``src/`` directory:

.. code-block:: console

    $ cd ~/ros2_ws/src
    $ git clone https://github.com/ros2/rosidl_buffer_backends.git
    $ git clone https://github.com/ros2/rosidl_buffer_backends_tutorials.git

Install the system dependencies:

.. code-block:: console

    $ cd ~/ros2_ws
    $ rosdep install --from-paths src --ignore-src -y \
        --skip-keys "fastcdr rti-connext-dds-7.7.0 urdfdom_headers qt6-svg-dev"

Build the CUDA backend first, source it, then build the demo:

.. code-block:: console

    $ colcon build --symlink-install --packages-up-to cuda_buffer_backend
    $ source install/setup.sh
    $ colcon build --symlink-install --packages-up-to robot_arm_demo
    $ source install/setup.sh

The intermediate ``source install/setup.sh`` is required so that the Torch
backend can discover the CUDA backend at CMake configure time and compile
its CUDA path.

Running
-------

The demo ships three launch files.

CUDA zero-copy (default)
^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: console

    $ ros2 launch robot_arm_demo robot_arm_demo.launch.py

The renderer and display processes negotiate the ``torch`` backend on top
of ``cuda``; the image payload never leaves GPU memory between the two
processes.

CPU transport
^^^^^^^^^^^^^

.. code-block:: console

    $ ros2 launch robot_arm_demo robot_arm_demo.launch.py use_cuda:=false

The subscription is not opted into any non-CPU backend, so the RMW falls
back to CPU serialisation and the publisher copies the frame back to host
memory before publishing.

Side-by-side comparison
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: console

    $ ros2 launch robot_arm_demo robot_arm_compare.launch.py

This starts both pipelines at once so you can compare their throughput
visually.

All three launch files accept ``headless:=true`` to run without a display
window (useful for benchmarking on a headless box).

Expected output
---------------

When the CUDA path is active, the display node logs the negotiated backend:

.. code-block:: console

    [display_node]: Received frame: backend=torch, size=31457280

When the path falls back to CPU, the same log line shows ``backend=cpu``
and the FPS drops accordingly as image size grows.

Benchmark results
-----------------

The demo's README includes reference numbers measured on a single machine
(inter-process, headless mode, RTX 3090, ``rmw_fastrtps_cpp``). You can
reproduce them with:

.. code-block:: console

    $ ros2 launch robot_arm_demo robot_arm_demo.launch.py \
        headless:=true width:=W height:=H use_cuda:=<true|false>

.. list-table::
   :widths: 20 20 20 20 20
   :header-rows: 1

   * - Resolution
     - Image size
     - Transport
     - FPS
     - Speedup
   * - 1920x1080
     - 7.9 MB
     - CUDA
     - 116.6
     - 3.3x
   * - 1920x1080
     - 7.9 MB
     - CPU
     - 35.5
     - --
   * - 2560x1440
     - 14.1 MB
     - CUDA
     - 90.6
     - 4.3x
   * - 2560x1440
     - 14.1 MB
     - CPU
     - 21.3
     - --
   * - 3840x2160
     - 31.6 MB
     - CUDA
     - 59.5
     - 5.8x
   * - 3840x2160
     - 31.6 MB
     - CPU
     - 10.3
     - --

The CUDA path maintains high throughput across resolutions because the
zero-copy IPC transfer only carries a handle, not the pixel data. The CPU
path must copy frames from GPU to host and serialise them through the
middleware, so throughput drops as image size grows. At 4K (31.6 MB per
frame) the CUDA backend is roughly 6x faster than the raw CPU path.

What to look at in the source
-----------------------------

Two files are worth reading to see exactly what an application does
differently on each side:

* The renderer's publisher code shows how ``torch_buffer_backend::allocate_msg``
  is used to produce a GPU-backed ``sensor_msgs::msg::Image`` and how the
  tensor is written in place via ``torch_buffer_backend::to_buffer``.
* The display's subscriber sets
  ``SubscriptionOptions::acceptable_buffer_backends = "torch"`` and branches
  on ``msg->data.get_backend_type()`` to either consume the data as a
  PyTorch tensor (zero-copy) or fall back to a host-side copy when the CPU
  path is in use.

Both files are reasonably short and make good reading after
:doc:`../../How-To-Guides/Using-Buffer-Backends`.

Where to go next
----------------

* :doc:`../../Concepts/Intermediate/About-Buffer-Backends` for the
  conceptual background.
* :doc:`../../How-To-Guides/Using-Buffer-Backends` for the user-facing
  guide applied to your own nodes.
* :doc:`../Advanced/Writing-a-Buffer-Backend` if you want to implement a
  backend for a different memory domain.
