Make sure you have a locale which supports ``UTF-8``.
If you are in a minimal environment (such as a docker container), the locale may be something minimal like ``C``.
We test with the following settings. However, it should be fine if you're using a different UTF-8 supported locale.

.. code-block:: bash

   locale  # check for UTF-8

   sudo dnf install langpacks-en glibc-langpack-en
   export LANG=en_US.UTF-8

   locale  # verify settings
