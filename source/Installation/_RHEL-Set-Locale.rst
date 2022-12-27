Asegúrate de tener una configuración regional que admita ``UTF-8``.
Si se encuentra en un entorno mínimo (como un contenedor docker), la configuración regional puede ser algo mínimo como ``C``.
Probamos con los siguientes ajustes. Sin embargo, debería estar bien si está utilizando una configuración regional compatible con UTF-8 diferente.

.. code-block:: bash

   locale  # check for UTF-8

   sudo dnf install langpacks-en glibc-langpack-en
   export LANG=en_US.UTF-8

   locale  # verify settings
