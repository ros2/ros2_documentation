
The ROS_DOMAIN_ID
=================

Overview
--------

As explained elsewhere, the default middleware that ROS 2 uses for communication is DDS.
In DDS, the primary mechanism for having different logical networks share a physical network is known as the Domain ID.
ROS 2 nodes on the same domain can freely discover and send messages to each other, while ROS 2 nodes on different domains cannot.
All ROS 2 nodes use domain ID 0 by default.
To avoid interference between different groups of computers running ROS 2 on the same network, a different domain ID should be set for each group.

Choosing a domain ID (short version)
------------------------------------

The text below explains the derivation of the range of domain IDs that should be used in ROS 2.
To skip that background and just choose a safe number, simply choose a domain ID between 0 and 101, inclusive.


Choosing a domain ID (long version)
-----------------------------------

The domain ID is used by DDS to compute the UDP ports that will be used for discovery and communication.
See `this article <https://community.rti.com/content/forum-topic/statically-configure-firewall-let-omg-dds-traffic-through>`__ for details on how the ports are computed.
Remembering our basic networking, the UDP port is an `unsigned 16-bit integer <https://en.wikipedia.org/wiki/User_Datagram_Protocol#Ports>`__.
Thus, the highest port number that can be allocated is 65535.
Doing some math with the formula in the article above, this means that the highest domain ID that can possibly be assigned is 232, while the lowest that can be assigned is 0.

Platform-specific constraints
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For maximum compatibility, some additional platform-specific constraints should be followed when choosing a domain ID.
In particular, it is best to avoid allocating domain IDs in the operating system's `ephemeral port range <https://en.wikipedia.org/wiki/Ephemeral_port>`__.
This avoids possible conflicts between the ports used by the ROS 2 nodes and other networking services on the computers.

Here are some platform-specific notes about ephemeral ports.

.. tabs::

   .. group-tab:: Linux

     By default, the Linux kernel uses ports 32768-60999 for ephemeral ports.
     This means that domain IDs 0-101 and 215-232 can be safely used without colliding with ephemeral ports.
     The ephemeral port range is configurable in Linux by setting custom values in ``/proc/sys/net/ipv4/ip_local_port_range``.
     If a custom ephemeral port range is used, the above numbers may have to be adjusted accordingly.

   .. group-tab:: macOS

     By default, the ephemeral port range on macOS is 49152-65535.
     This means that domain IDs 0-166 can be safely used without colliding with ephemeral ports.
     The ephemeral port range is configurable in macOS by setting custom sysctl values for ``net.inet.ip.portrange.first`` and ``net.inet.ip.portrange.last``.
     If a custom ephemeral port range is used, the above numbers may have to be adjusted accordingly.

   .. group-tab:: Windows

     By default, the ephemeral port range on Windows is 49152-65535.
     This means that domain IDs 0-166 can be safely used without colliding with ephemeral ports.
     The ephemeral port range is configurable in Windows by `using netsh <https://docs.microsoft.com/en-us/troubleshoot/windows-server/networking/default-dynamic-port-range-tcpip-chang>`__.
     If a custom ephemeral port range is used, the above numbers may have to be adjusted accordingly.

Participant constraints
^^^^^^^^^^^^^^^^^^^^^^^

For each ROS 2 process running on a computer, one DDS "participant" is created.
Since each DDS participant takes up two ports on the computer, running more than 120 ROS 2 processes on one computer may spill over into other domain IDs or the ephemeral ports.

To see why, consider the domain IDs 1 and 2.

- Domain ID 1 uses port 7650 and 7651 for multicast.
- Domain ID 2 uses port 7900 and 7901 for multicast.
- When creating the 1st process (zeroth participant) in domain ID 1, the ports 7660 and 7661 are used for unicast.
- When creating the 120th process (119th participant) in domain ID 1, the ports 7898 and 7899 are used for unicast.
- When creating the 121st process (120th participant) in domain ID 1, the ports 7900 and 7901 are used for unicast and overlap with domain ID 2.

If it is known that the computer will only ever be on a single domain ID at a time, and the domain ID is low enough, it is safe to create more ROS 2 processes than this.

When choosing a domain ID that is near the top of the range of platform-specific domain IDs, one additional constraint should be considered.

For instance, assume a Linux computer with a domain ID of 101:

- The zero'th ROS 2 process on the computer will connect to ports 32650, 32651, 32660, and 32661.
- The first ROS 2 process on the computer will connect to ports 32650, 32651, 32662, and 32663.
- The 53rd ROS 2 process on the computer will connect to ports 32650, 32651, 32766, and 32767.
- The 54th ROS 2 process on the computer will connect to ports 32650, 32651, 32768, and 32769, running into the ephemeral port range.

Thus the maximum number of processes that should be created when using domain ID 101 on Linux is 54.
Similarly, the maximum number of processes that should be created when using domain ID 232 on Linux is 63, as the maximum port number is 65535.

The situation is similar on macOS and Windows, though the numbers are different.
On macOS and Windows, when choosing a domain ID of 166 (the top of the range), the maximum number of ROS 2 processes that can be created on a computer before running into the ephemeral port range is 120.

Domain ID to UDP Port Calculator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. raw:: html

    <table>
      <tr>
        <td style="text-align: right; vertical-align: middle;"><label>Domain ID:</label></td>
        <td><input type="number" min="0" max="232" size="3" class="display" value="0" id="domainID" onChange="calculate(this.value)"/></td>
      </tr>
      <tr>
        <td style="text-align: right; vertical-align: middle;"><label>Participant ID:</label></td>
        <td><input type="number" min="0" size="3" class="display" value="0" id="participantID" onChange="calculate(this.value)"/></td>
      </tr>
    </table>
    <hr/>
    <table>
      <tr>
        <td style="text-align: right; vertical-align: middle;"><label>Discovery Multicast Port:</label></td>
        <td><input type="text" size="5" class="discoveryMulticastPort" disabled/></td>
      </tr>
      <tr>
        <td style="text-align: right; vertical-align: middle;"><label>User Multicast Port:</label></td>
        <td><input type="text" size="5" class="userMulticastPort" disabled/></td>
      </tr>
      <tr>
        <td style="text-align: right; vertical-align: middle;"><label>Discovery Unicast Port:</label></td>
        <td><input type="text" size="5" class="discoveryUnicastPort" disabled/></td>
      </tr>
      <tr>
        <td style="text-align: right; vertical-align: middle;"><label>User Unicast Port:</label></td>
        <td><input type="text" size="5" class="userUnicastPort" disabled/></td>
      </tr>
    </table>
    <br/>
    <br/>

    <script type="text/javascript">
      window.addEventListener('load', (event) => {
         calculate(event);
      });
      const discoveryMcastPort = document.querySelector('.discoveryMulticastPort');
      const userMcastPort = document.querySelector('.userMulticastPort');
      const discoveryUnicastPort = document.querySelector('.discoveryUnicastPort');
      const userUnicastPort = document.querySelector('.userUnicastPort');

      const domainID = document.getElementById('domainID');
      const participantID = document.getElementById('participantID');

      // calculate function
      function calculate(event) {
        const d0 = 0;
        const d2 = 1;
        const d1 = 10;
        const d3 = 11;
        const PB = 7400;
        const DG = 250;
        const PG = 2;

        discoveryMcastPort.value = PB + (DG * domainID.value) + d0;
        userMcastPort.value = PB + (DG * domainID.value) + d2;
        discoveryUnicastPort.value = PB + (DG * domainID.value) + d1 + (PG * participantID.value);
        userUnicastPort.value = PB + (DG * domainID.value) + d3 + (PG * participantID.value);
      }
    </script>
