.. redirect-from::

  Installation/Fedora-Development-Setup

Fedora (fuentes)
================

¿Cómo configurar el entorno de desarrollo?
------------------------------------------

Se requieren las siguientes dependencias del sistema para compilar ROS 2 en Fedora. Se pueden instalar con ``dnf`` de la siguiente manera:

.. code-block:: bash

   sudo dnf install \
     cmake \
     cppcheck \
     eigen3-devel \
     gcc-c++ \
     liblsan \
     libXaw-devel \
     libyaml-devel \
     make \
     opencv-devel \
     patch \
     python3-colcon-common-extensions \
     python3-coverage \
     python3-devel \
     python3-empy \
     python3-nose \
     python3-pip \
     python3-pydocstyle \
     python3-pyparsing \
     python3-pytest \
     python3-pytest-cov \
     python3-pytest-mock \
     python3-pytest-runner \
     python3-rosdep \
     python3-setuptools \
     python3-vcstool \
     poco-devel \
     poco-foundation \
     python3-flake8 \
     python3-flake8-import-order \
     redhat-rpm-config \
     uncrustify \
     wget


Una vez hecho esto, puedes seguir el resto de las :ref:`instrucciones <Rolling_rhel-dev-get-ros2-code>` para obtener y compilar ROS 2.

