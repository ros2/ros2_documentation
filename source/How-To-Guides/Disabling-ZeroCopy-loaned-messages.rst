.. _ZeroCopyLoanedMessages:

Disabling Zero Copy Loaned Messages
===================================

.. contents:: Contents
   :depth: 1
   :local:

See the `Loaned Messages <https://design.ros2.org/articles/zero_copy.html>`__ article for details on how loaned messages work.

How to disable Loaned Messages
------------------------------

By default, *Loaned Messages* will try to borrow the memory from underlying middleware if it supports *Loaned Messages*.
The ``ROS_DISABLE_LOANED_MESSAGES`` environment variable can be used to disable *Loaned Messages*, and fallback to normal publisher and subscription behavior, without any code changes or middleware configuration.
You can set the environment variable with the following command:

.. tabs::

   .. group-tab:: Linux

      .. code-block:: console

        export ROS_DISABLE_LOANED_MESSAGES=1

      To maintain this setting between shell sessions, you can add the command to your shell startup script:

      .. code-block:: console

        echo "export ROS_DISABLE_LOANED_MESSAGES=1" >> ~/.bashrc

   .. group-tab:: macOS

      .. code-block:: console

        export ROS_DISABLE_LOANED_MESSAGES=1

      To maintain this setting between shell sessions, you can add the command to your shell startup script:

      .. code-block:: console

        echo "export ROS_DISABLE_LOANED_MESSAGES=1" >> ~/.bash_profile

   .. group-tab:: Windows

      .. code-block:: console

        set ROS_DISABLE_LOANED_MESSAGES=1

      If you want to make this permanent between shell sessions, also run:

      .. code-block:: console

        setx ROS_DISABLE_LOANED_MESSAGES 1
