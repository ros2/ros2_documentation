Installing Connext security plugins
===================================

The Connext DDS Libraries are included with ROS 2 under a `non-commercial
license <https://www.rti.com/ncl>`__ and do not include the security
plug-in libraries. These libraries are available in the commercial,
university and research license versions of RTI Connext DDS Pro, which
is bundled with tools for system debugging, monitoring, record/replay,
etc.

The Connext DDS Evaluation Version (5.3.1) includes the security plugins, and can be downloaded via options available for `university, purchase or evaluation <Install-Connext-University-Eval>`.

A video walk-thru of this installation (tools and security plug-ins) is
available
`here <https://www.rti.com/gettingstarted/installwindows_secure>`__ at
the RTI website. The steps are:

**Install Connext DDS Pro (Host)**
This is a host-specific installer application (for Windows, Linux, MacOS) to install a 'Host' bundle which includes the Launcher, tools, and other software services.
At the end of the installation, the RTI 'Launcher' program will be started.
The Launcher is used to install target libraries, security plugins, and other layered services.

**Use the Package Installer in Launcher**

.. figure:: https://cdn2.hubspot.net/hub/1754418/file-3649043118-png/blog-files/launchermacos.png
   :alt: Launcher Image

   Launcher Image

The 'RTI Package Installer' is used to install '.rtipkg' files -- target
libraries, security plug-ins, etc. Open the Package Installer and select
all of the .rtipkg files that were included in the Connext DDS Secure
bundle for installation:

 * Target Libraries - such as: rti\_connext\_dds-[version]-pro-target-[toolchain].rtipkg
 * Security Plugin Host - such as: rti\_security\_plugins-[version]-host-[toolchain].rtipkg
 * Security Plugin Target - such as: rti\_security\_plugins-[version]-target-[toolchain].rtipkg
 * OpenSSL Host - such as: openssl-1.0.2x-[version]-host-[toolchain].rtipkg

**Extract and Install OpenSSL**
This is included as an archive (.zip or
otherwise) and can be simply extracted and copied to a convenient
location on your host computer. As a suggestion, this could also be
installed into the 'rti\_connext\_dds-[version]' directory in your home
directory space (this was created during installation of the RTI host
tools). Note: this directory location may need to be placed in your PATH
environment variable.
See the `RTI Security Plugins Getting Started Guide <https://community.rti.com/static/documentation/connext-dds/5.3.1/doc/manuals/connext_dds/dds_security/RTI_SecurityPlugins_GettingStarted.pdf>`__ for more information.

Installation complete.
