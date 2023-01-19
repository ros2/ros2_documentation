Asegúrate de tener una configuración regional que admita ``UTF-8``.
Si te encuentra en un entorno mínimo (como un contenedor docker), la configuración regional puede ser algo mínimo como ``POSIX``.
Hemos probado con los siguientes ajustes. Sin embargo, debería también ser correcto utilizar una configuración regional compatible con UTF-8 diferente.

.. code-block:: bash

   locale  # check for UTF-8

   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8

   locale  # verify settings
