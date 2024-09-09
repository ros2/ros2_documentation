.. redirect-from::

   Installation/Linux-Install-Debians
   Installation/Ubuntu-Install-Debians

Ubuntu (deb packages)
=====================

.. contents:: Table of Contents
   :depth: 2
   :local:

ROS 2 {DISTRO_TITLE_FULL} 的 Deb packages 目前在 Ubuntu Jammy (22.04) 上可用.
The target platforms are defined in `REP 2000 <https://ros.org/reps/rep-2000.html>`__.

相关资源
---------

* 状态页:
  * ROS 2 {DISTRO_TITLE} (Ubuntu Jammy): `amd64 <http://repo.ros2.org/status_page/ros_{DISTRO}_default.html>`__\ , `arm64 <http://repo.ros2.org/status_page/ros_{DISTRO}_ujv8.html>`__
* `Jenkins Instance <http://build.ros2.org/>`__
* `源码仓库 <http://repo.ros2.org>`__


设置地区
----------

.. include:: _Ubuntu-Set-Locale.rst

.. _linux-install-debians-setup-sources:

设置 apt 源仓库
----------------

.. include:: _Apt-Repositories.rst

.. _linux-install-debs-install-ros-2-packages:

安装 ROS 2
----------------------

设置好仓库后，请更新 apt 仓库缓存。

.. code-block:: bash

   sudo apt update

.. include:: _Apt-Upgrade-Admonition.rst

.. warning::

   Due to early updates in Ubuntu 22.04 it is important that ``systemd`` and ``udev``-related packages are updated before installing ROS 2.
   The installation of ROS 2's dependencies on a freshly installed system without upgrading can trigger the **removal of critical system packages**.

   Please refer to `ros2/ros2#1272 <https://github.com/ros2/ros2/issues/1272>`_ and `Launchpad #1974196 <https://bugs.launchpad.net/ubuntu/+source/systemd/+bug/1974196>`_ for more information.

桌面版安装(推荐)包含: ROS, RViz, demos, 教程.

.. code-block:: bash

   sudo apt install ros-{DISTRO}-desktop

基本版安装包含：通信库、消息包、命令行工具.
没有 GUI 工具.

.. code-block:: bash

   sudo apt install ros-{DISTRO}-ros-base

开发工具包: 编译器和其他工具用于构建 ROS 包.

.. code-block:: bash

   sudo apt install ros-dev-tools

环境配置
-----------------

Sourcing 配置脚本
^^^^^^^^^^^^^^^^^^^^^^^^^

source 以下脚本之一以设置环境变量（译者注： 取决于你使用何种shell）:

.. code-block:: bash

   # Replace ".bash" with your shell if you're not using bash
   # Possible values are: setup.bash, setup.sh, setup.zsh
   source /opt/ros/{DISTRO}/setup.bash

尝试一些样例以验证安装
---------------------

Talker-listener
^^^^^^^^^^^^^^^

如果你之前安装了 ``ros-{DISTRO}-desktop``，那你可以运行这个样例.

在一个 terminal 中，source 配置文件然后运行 C++ ``talker``:

.. code-block:: bash

   source /opt/ros/{DISTRO}/setup.bash
   ros2 run demo_nodes_cpp talker

在另一个 terminal 中，source 配置文件然后运行 Python ``listener``:

.. code-block:: bash

   source /opt/ros/{DISTRO}/setup.bash
   ros2 run demo_nodes_py listener

你应该看到 ``talker`` 说它在 ``Publishing`` 消息，而 ``listener`` 说 ``I heard`` 这些消息.
这验证了 C++ 和 Python API 都正常工作.
说明你做得很好！

安装完成后下一步做什么
---------------------------
继续查看 :doc:`tutorials and demos <../../Tutorials>` 来配置环境, 创建你自己的工作区(workspace)和包(packages), 学习 ROS 2 核心概念.

使用 the ROS 1 bridge
----------------------
The ROS 1 bridge 可以将 topic 从 ROS 1 传递至 ROS 2，反之亦然. 请查看 `此文档 <https://github.com/ros2/ros1_bridge/blob/master/README.md>`__ 以知晓如何构建和使用 the ROS 1 bridge.

其它 RMW 实现 (可选内容)
-----------------------------------------
The default middleware that ROS 2 uses is ``Fast DDS``, but the middleware (RMW) can be replaced at runtime.
See the :doc:`guide <../How-To-Guides/Working-with-multiple-RMW-implementations>` on how to work with multiple RMWs.

异常排查
---------------

异常排查有关的技巧和文档 :doc:`here <../How-To-Guides/Installation-Troubleshooting>`.

卸载
---------

如果你需要卸载 ROS 2 或者在已经安装了二进制包后想切换到源码安装，运行以下命令:

.. code-block:: bash

  sudo apt remove ~nros-{DISTRO}-* && sudo apt autoremove

你可能还想移除仓库源:

.. code-block:: bash

  sudo rm /etc/apt/sources.list.d/ros2.list
  sudo apt update
  sudo apt autoremove
  # Consider upgrading for packages previously shadowed.
  sudo apt upgrade
