.. _InstallationGuide:
.. _RollingInstall:

安装
============

可行的 ROS 2 {DISTRO_TITLE_FULL} 安装方式:

.. toctree::
   :hidden:
   :glob:

   Installation/Ubuntu-Install-Debs
   Installation/Windows-Install-Binary
   Installation/RHEL-Install-RPMs
   Installation/Alternatives
   Installation/Maintaining-a-Source-Checkout
   Installation/Testing
   Installation/DDS-Implementations

从二进制包安装
---------------

只有在 Tier 1 系统上才会为 ROS 2 提供二进制包，这些系统列在 `REP-2000 <https://www.ros.org/reps/rep-2000.html#rolling-ridley-june-2020-ongoing>`__ 中。
如果您的操作系统中不在下述列表中，您可能需要从源码构建或使用 :doc:`容器解决方案 <How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers>` 在您的平台上运行 ROS 2。

我们为以下平台提供 ROS 2 二进制包:

* Ubuntu Linux - Jammy Jellyfish (22.04)

  * :doc:`deb packages <Installation/Ubuntu-Install-Debs>` (推荐)
  * :doc:`binary archive <Installation/Alternatives/Ubuntu-Install-Binary>`

* RHEL 8

  * :doc:`RPM packages <Installation/RHEL-Install-RPMs>` (推荐)
  * :doc:`binary archive <Installation/Alternatives/RHEL-Install-Binary>`

* Windows 10

  * :doc:`Windows Binary (VS 2019) <Installation/Windows-Install-Binary>`

.. _building-from-source:

从源码构建并安装
--------------------

我们支持在以下平台上从源码构建 ROS 2:

* :doc:`Ubuntu Linux 22.04 <Installation/Alternatives/Ubuntu-Development-Setup>`
* :doc:`Windows 10 <Installation/Alternatives/Windows-Development-Setup>`
* :doc:`RHEL-8 <Installation/Alternatives/RHEL-Development-Setup>`
* :doc:`macOS <Installation/Alternatives/macOS-Development-Setup>`

我应该选择哪种安装方式?
--------------------------------

从二进制包或从源码安装都是完全可用的 ROS 2 安装方式。
这两种方式的区别取决于您打算如何使用 ROS 2。

**二进制包**适用于一般用户，提供了 ROS 2 的已构建安装。
这对于想要立即开始使用 ROS 2 的人来说非常方便。

Linux 用户有两种安装二进制包的方式:

- Packages (debs or RPMS, depending on the platform)
- binary archive

从包(Packages)安装是推荐的方法，因为它会自动安装必要的依赖项，并且还会随着常规系统更新一起更新。
但是，您需要 root 权限才能安装 Debian 包。
如果您没有 root 权限，二进制存档(binary archive)则是最佳选择。

Windows 用户只能选择二进制存档。
(deb 包仅适用于 Ubuntu/Debian)

**从源码构建**适用于想要更改或屏蔽部分 ROS 2 核心的开发人员。
它还适用于不支持二进制的平台。
从源码构建还可以让您安装 ROS 2 的最新版本。

向 ROS 2 core 贡献代码?
^^^^^^^^^^^^^^^^^^^^^^^^^^^

如果您计划直接向 ROS 2 核心包贡献代码，您可以安装 :doc:`最新的开发源码 <Installation/Alternatives/Latest-Development-Setup>`，这与 :ref:`Rolling 发行版 <rolling_distribution>` 共享安装说明。
