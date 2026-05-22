Supplementing custom rosdep keys
================================

.. contents:: Contents
    :depth: 2
    :local:

Overview and motivation
-----------------------

As explained in :doc:`../Intermediate/Rosdep`, ``rosdep`` looks for rosdep keys in ``package.xml`` files and maps them to packages to be installed for the ROS distribution and OS in use.
Anyone can request for new rosdep keys to be added by `contributing to rosdistro <https://github.com/ros/rosdistro/blob/master/CONTRIBUTING.md#rosdep-rules-contributions>`_.
This is the preferred course of action when you have some dependency (like a ``apt`` or ``pip`` package) that you wish to be able to install via ``rosdep``.

However, there are many cases when contributing your keys directly might be difficult.
For example, if the dependency

1. is not available on the target distributions default APT (or pip) repositories
2. is a proprietary library
3. is some niche library which is useful only for you or your organization
4. is a ROS package you :doc:`built and packaged yourself <../../How-To-Guides/Building-a-Custom-Deb-Package>`, but don't wish to share with the broader ROS community

While the option exists to :doc:`fork rosdistro <../../How-To-Guides/Using-Custom-Rosdistro>` in its entirety, this might be overkill if you just want to keep using a official ROS distribution as usual, only with some extra rosdep keys defined on top.
This tutorial explains how to achieve this.

As a word of warning though, please don't use this indiscriminately.
It can cause very hard to debug problems because of binary incompatibilities which can manifest in silent failures, unexplained crashes, or data corruption.
And if you're asking anyone for help on a system with this enabled, make sure to explain everything that's being added or overridden.

Preliminaries: How ``rosdep`` fetches rosdep keys
-------------------------------------------------

In order to get a good understanding of what we're about to do, let's first explore some relevant details about how ``rosdep`` works.

``rosdep`` is similar to other tools like ``apt`` that use a sources list to maintain a local index.
These sources are stored in ``/etc/ros/rosdep/sources.list.d``.
This is similar to how apt stores repositories in ``/etc/apt/sources.list.d``.

By default (as part of first-time setup, ``rosdep init``), you only have a single sources file: ``/etc/ros/rosdep/sources.list.d/20-default.list``.
Inspecting its contents, you will see entries like these:

.. code-block:: console

   $ cat /etc/ros/rosdep/sources.list.d/20-default.list
   ...
   yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
   yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
   ...

These entries are what dictates where ``rosdep`` fetches rosdep keys and their mappings (rosdep **rules**) from when ``rosdep update`` is called.
When called, ``rosdep`` compiles the relevant contents from all declared entries in all sources files into a local cached index.
This local index is then used when installing or looking up ("resolving") rosdep keys.

For example, the fact that the first entry (``base.yaml``) defines the ``libopencv-dev`` key (see `here <https://github.com/ros/rosdistro/blob/72f24d6/rosdep/base.yaml#L5240-L5252>`_) is what allows ``rosdep`` to resolve it:

.. code-block:: console

   $ rosdep where-defined libopencv-dev
   https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
   $ rosdep resolve libopencv-dev
   #apt
   libopencv-dev

In summary, it allows ``rosdep`` to resolve the ``libopencv-dev`` key to the ``apt`` package of the same name.

Note that, from the output above, we can deduce that the command was run on a Ubuntu or Debian OS.
On RHEL, the key instead resolves to the DNF package ``opencv-devel``:

.. code-block:: console

   $ rosdep resolve libopencv-dev --os=rhel:9
   #dnf
   opencv-devel

Extending ``rosdep`` with a custom sources file
-----------------------------------------------

The above hopefully makes it clear what needs to be done to get ``rosdep`` to understand new keys: add a new custom sources file!

As a toy example, let's add a new sources file telling ``rosdep`` to fetch keys from a YAML file stored on the local machine.
Fire up your favorite text editor and write the following into ``/etc/ros/rosdep/sources.list.d/30-custom.list`` (the editor will need to be launched with root privileges, e.g. via ``sudo``):

.. code-block:: yaml

  yaml file:///etc/ros/rosdep/custom_rules.yaml

Now write the following into ``/etc/ros/rosdep/custom_rules.yaml``:

.. code-block:: yaml

  awesome_library:
    ubuntu: [awesome_library]
  that_other_library:
    ubuntu:
      pip:
        packages: [another_library]

This defines two new rosdep rules:

1. The key ``awesome_library``, defined only for Ubuntu, mapping to the ``apt`` package of the same name
2. The key ``that_other_library``, defined only for Ubuntu, mapping to the ``pip`` package named ``another_library``

After running ``rosdep update``, ``rosdep`` will detect the new ``30-custom.list``, prompting it to scan the contents of the ``custom_rules.yaml`` file.
Now ``rosdep`` is set up to recognize these new keys and what they should map to:

.. code-block:: console

   $ rosdep resolve awesome_library
   #apt
   awesome_library
   $ rosdep resolve that_other_library
   #pip
   another_library

Now all you have to do is add ``<depend>awesome_library</depend>`` to your ROS packages ``package.xml``, and ``rosdep`` will know how to install the dependency!

Closing remarks
---------------

The toy example above only hints at what is possible with custom rosdep keys.

- **Is your dependency an APT package hosted in a third party PPA?**
  Not a problem.
  Since all ``rosdep`` does is converting the key to a ``apt install`` invocation, APT will have no problem installing the package (provided you have added the PPA).
- **Is your dependency a pip package hosted in a third party index?**
  Add the index to your ``pip.conf`` and you're good to go.
- **Sources files don't have to point to files on the local machine.**
  Both ``file://`` and ``https://`` syntax is supported (on Linux, absolute paths start with ``/``, which leads to a triple slash like ``file:///etc/rosdep/my.file``).
- **Sources are loaded in alphabetical order.**
  If you add a conflicting rule in the 30 prefix it will not be used.
  If you created a sources file with a 10 prefix it will override packages in the default list (prefix 20).
  If you're using packages installed from binary repositories it's highly recommended to not override dependency declarations as it will likely cause binary incompatibilities which can be very hard to debug.
- **Merging keys is not possible.**
  It is not possible to e.g. only add a ``fedora`` installation rule to an existing rosdep key.
  Depending on the load order, such rule would either be ignored, or it would completely override the whole rosdep key, removing all other installers.

Further reading
---------------

- https://docs.ros.org/en/independent/api/rosdep/html/rosdep_yaml_format.html
- https://docs.ros.org/en/independent/api/rosdep/html/contributing_rules.html
