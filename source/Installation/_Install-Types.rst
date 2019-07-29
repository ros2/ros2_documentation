
Which install should you choose?
--------------------------------

Installing from binary packages or from source will both result in a fully-functional and usable ROS 2 install.
Differences between the options depend on what you plan to do with ROS 2.

**Binary packages** are for general use and provide an already-built install of ROS 2.
This is great for people who want to dive in and start using ROS 2 as-is, right away.

Linux users have two further options:

- Debian packages. A combination of syncs and patch releases.
  Semi-frequent syncs incorporate changes to higher-level packages.
  Less-frequent patch releases incorporate changes to ROS 2's core packages along with changes from recent syncs.

- "fat" archive. Archived code of most recent patch release (won't include syncs since the last patch release).

Installing from Debian packages is the recommended method.
It's more convenient and updates alongside regular system updates.
However, you need sudo access in order to install Debian packages.
If you don't have sudo access, the "fat" archive is the next best choice.

OS X and Windows users who choose to install from binary packages only have the "fat" archive option (Debian packages are exclusive to Ubuntu/Debian).

**Building from source** is meant for developers looking to alter or explicitly omit parts of ROS 2's base.

.. TODO: add reference to "General Install" instructions "...if you don't see your operating system"
