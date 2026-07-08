GPU buffer transport with ``rosidl::Buffer``
============================================

.. contents:: Table of Contents
   :depth: 2
   :local:

Background
----------

:doc:`rosidl::Buffer backends <About-Buffer-Backends>`
let ROS 2 messages carry large payloads (images, tensors, ...) in
vendor-specific memory domains such as GPU memory, with the descriptor
round-trip handled transparently by the RMW.

This demo exercises the full pipeline using the community-maintained CUDA
backend and the ``torch_conversions`` helper library.
It publishes ``tensor_msgs/msg/ExperimentalTensor`` frames between two
processes either over a zero-copy CUDA path or over the traditional
CPU-serialised path, and compares throughput at several resolutions.

The underlying demo is maintained in
`ros2/rosidl_buffer_backends_tutorials
<https://github.com/ros2/rosidl_buffer_backends_tutorials>`__; the package
name is ``robot_arm_demo``.

What the demo does
------------------

``robot_arm_demo`` renders an SDF-based pencil-sketch robot arm animation
entirely on the GPU via LibTorch tensor operations, publishes BGRA frames
as ``tensor_msgs/msg/ExperimentalTensor``, and displays them in an
SDL2/OpenGL window with CUDA-GL interop.
Two processes are involved:

#. ``renderer_node`` -- renders BGRA frames on the GPU using LibTorch
   operations, copies them into an ``ExperimentalTensor`` message with
   ``torch_conversions``, and publishes that message.
#. ``display_node`` -- subscribes to the tensor topic, wraps the received
   message as an ``at::Tensor`` with ``torch_conversions``, displays frames,
   and reports FPS.

Two transport modes are compared:

* **CUDA** -- the ``ExperimentalTensor.data`` field is backed by the ``cuda``
  buffer backend.
  The bytes never leave GPU memory: the descriptor on the wire carries a small reference that lets the subscriber re-attach to the publisher-owned CUDA allocation.
* **CPU** -- the frame is rendered on the GPU, copied back to host memory
  with ``cudaMemcpy``, and then serialised through the RMW as a regular
  ``uint8[]``.
  The default CPU buffer backend is used.

Both modes render on the GPU; the only difference is the transport path,
making this a clean comparison of zero-copy CUDA IPC versus traditional
CPU-serialised image transport.

Prerequisites
-------------

This demo is not part of the ROS 2 binary distribution; it lives in its own
repository and has GPU-specific dependencies.
You need:

* A CUDA-capable GPU and the CUDA Toolkit (>= 11.8).
* SDL2, GLEW, OpenGL, X11 development packages.
* A ROS 2 Lyrical Luth or later source workspace.
  See the :doc:`Installation instructions <../../../../Get-Started/Installation>` for the
  canonical source-build flow.
* ``rmw_fastrtps_cpp`` for the non-CPU buffer path.

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

Build ``torch_conversions`` and its CUDA transport dependency first, source
the workspace, then build the demo:

.. code-block:: console

    $ colcon build --symlink-install --packages-up-to cuda_buffer_backend
    $ source install/setup.sh
    $ colcon build --symlink-install --packages-up-to robot_arm_demo
    $ source install/setup.sh

The intermediate ``source install/setup.sh`` is required so that
``torch_conversions`` can discover ``cuda_buffer`` at CMake configure time and
compile its CUDA path.

Running
-------

The demo ships three launch files.
Run them with ``RMW_IMPLEMENTATION=rmw_fastrtps_cpp`` when exercising the
CUDA path.

CUDA zero-copy (default)
^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: console

    $ export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    $ ros2 launch robot_arm_demo robot_arm_demo.launch.py

The renderer and display processes negotiate the ``cuda`` backend for the
tensor message's data field; the payload never leaves GPU memory between the
two processes.

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

    [display_node]: Received frame: backend=cuda, size=31457280

When the path falls back to CPU, the same log line shows ``backend=cpu``
and the FPS drops accordingly as image size grows.

Benchmark results
-----------------

The demo's README includes reference numbers measured on a single machine
(inter-process, headless mode, RTX 3090, ``rmw_fastrtps_cpp``).
You can reproduce them with:

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
zero-copy IPC transfer only carries a handle, not the pixel data.
The CPU path must copy frames from GPU to host and serialise them through
the middleware, so throughput drops as image size grows.
At 4K (31.6 MB per frame) the CUDA backend is roughly 6x faster than the
raw CPU path.

What to look at in the source
-----------------------------

Two files are worth reading to see exactly what an application does
differently on each side:

* The renderer's publisher code shows how
  ``torch_conversions::allocate_tensor_msg`` and
  ``torch_conversions::to_tensor_msg`` produce an
  ``ExperimentalTensor`` whose ``data`` field is CUDA-backed.
* The display's subscriber sets
  ``SubscriptionOptions::acceptable_buffer_backends = "any"`` for the CUDA
  path and uses ``torch_conversions::from_input_tensor_msg`` to consume the
  message as a PyTorch tensor.

Both files are reasonably short and make good reading after
:doc:`Using-Buffer-Backends`.

Where to go next
----------------

* :doc:`About-Buffer-Backends` for the
  conceptual background.
* :doc:`Using-Buffer-Backends` for the user-facing
  guide applied to your own nodes.
* :doc:`Writing-a-Buffer-Backend` if you want to implement a
  backend for a different memory domain.
