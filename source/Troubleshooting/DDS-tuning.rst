DDS tuning information
======================

This page provides some guidance on parameter tunings that were found to address issues faced while using various DDS implementations on Linux in real-world situations.
It is possible that the issues we identified on Linux or while using one vendor may occur for other platforms and vendors not documented here.

The recommendations below are starting points for tuning; they worked for specific systems and environments, but the tuning may vary depending on a number of factors.
You may need to increase or decrease values while debugging relative to factors like message size, network topology, etc.

It is important to recognize that tuning parameters can come at a cost to resources, and may affect parts of your system beyond the scope of the desired improvements.
The benefits of improving reliability should be weighed against any detriments for each individual case.

.. _cross-vendor-tuning:

Cross-vendor tuning
-------------------

**Issue:** Sending data over lossy (usually WiFi) connections becomes problematic when some IP fragments are dropped, possibly causing the kernel buffer on the receiving side to become full.

When a UDP packet is missing at least one IP fragment, the rest of the received fragments fill up the kernel buffer.
By default, the Linux kernel will time out after 30s of trying to recombine packet fragments.
Since the kernel buffer is full at this point (default size is 256KB), no new fragments can come in, and so the connection will seemingly "hang" for long periods of time.

This issue is generic across all DDS vendors, so the solutions involve adjusting kernel parameters.

**Solution:** Use best-effort QoS settings instead of reliable.

Best-effort settings reduce the amount of sent data since it will not resend incomplete samples and therefore will flood the network less than reliable.
If the kernel buffer for IP fragments gets full, though, the symptom is still the same (blocking for 30s).
This solution should improve the issue somewhat without having to adjust parameters.

**Solution:** Reduce the value of the ``ipfrag_time`` parameter.

``net.ipv4.ipfrag_time / /proc/sys/net/ipv4/ipfrag_time`` (default 30s) :
Time in seconds to keep an IP fragment in memory.

Reduce the value, for example, to 3s, by running:

.. code-block:: console

    sudo sysctl net.ipv4.ipfrag_time=3

Reducing this parameter’s value also reduces the window of time where no fragments are received.
The parameter is global for all incoming fragments, so the feasibility of reducing its value needs to be considered for every environment.

**Solution:** Increase the value of the ``ipfrag_high_thresh`` parameter.

``net.ipv4.ipfrag_high_thresh / /proc/sys/net/ipv4/ipfrag_high_thresh`` (default: 262144 bytes):
Maximum memory used to reassemble IP fragments.

Increase the value, for example, to 128MB, by running:

.. code-block:: console

    sudo sysctl net.ipv4.ipfrag_high_thresh=134217728     # (128 MB)

Significantly increasing this parameter’s value is an attempt to ensure that the buffer never becomes completely full.
However, the value would likely have to be significantly high to hold all data received during the time window of ``ipfrag_time``, assuming every UDP packet lacks one fragment.

Fast RTPS tuning
----------------

**Issue:** Fast RTPS floods the network with large pieces of data or fast-published data when operating over WiFi.

See the solutions under :ref:`Cross-vendor tuning <cross-vendor-tuning>`.

Cyclone DDS tuning
------------------

**Issue:** Cyclone DDS is not delivering large messages reliably, despite using reliable settings and transferring over a wired network.

This issue should be `addressed soon <https://github.com/eclipse-cyclonedds/cyclonedds/issues/484>`_.
Until then, we’ve come up with the following solution (debugged using `this test program <https://github.com/jacobperron/pc_pipe>`_):

**Solution:** Increase the maximum Linux kernel receive buffer size and the minimum socket receive buffer size that Cyclone uses.

*Adjustments to solve for a 9MB message:*

Set the maximum receive buffer size, ``rmem_max``, by running:

 .. code-block:: console

    sudo sysctl -w net.core.rmem_max=2147483647

Or permanently set it by editing the ``/etc/sysctl.d/10-cyclone-max.conf`` file to contain:

 .. code-block:: console

    net.core.rmem_max=2147483647

Next, to set the minimum socket receive buffer size that Cyclone requests, write out a configuration file for Cyclone to use while starting, like so:

.. code-block:: xml

  <?xml version="1.0" encoding="UTF-8" ?>
  <CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config
  https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
      <Domain id="any">
          <Internal>
              <MinimumSocketReceiveBufferSize>10MB</MinimumSocketReceiveBufferSize>
          </Internal>
      </Domain>
  </CycloneDDS>

Then, whenever you are going to run a node, set the following environment variable:

.. code-block:: console

    CYCLONEDDS_URI=file:///absolute/path/to/config_file.xml

RTI Connext tuning
------------------

**Issue:** Connext is not delivering large messages reliably, despite using reliable settings and transferring over a wired network.

**Solution:** This `Connext QoS profile <https://github.com/jacobperron/pc_pipe/blob/master/etc/ROS2TEST_QOS_PROFILES.xml>`_, along with increasing the ``rmem_max`` parameter.

Set the maximum receive buffer size, ``rmem_max``, by running:

 .. code-block:: console

    sudo sysctl -w net.core.rmem_max=4194304

By tuning ``net.core.rmem_max`` to 4MB in the Linux kernel, the QoS profile can produce truly reliable behavior.

This configuration has been proven to reliably deliver messages via SHMEM|UDPv4, and with just UDPv4 on a single machine.
A multi-machine configuration was also tested with ``rmem_max`` at 4MB and at 20MB (two machines connected with 1Gbps ethernet), with no dropped messages and average message delivery times of 700ms and 371ms, respectively.

Without configuring the kernel’s ``rmem_max``, the same Connext QoS profile took up to 12 seconds for the data to be delivered.
However, it always at least managed to complete the delivery.

**Solution:** Use the `Connext QoS profile <https://github.com/jacobperron/pc_pipe/blob/master/etc/ROS2TEST_QOS_PROFILES.xml>`_ *without* adjusting ``rmem_max``.

The ROS2TEST_QOS_PROFILES.xml file was configured using RTI’s documentation on `configuring flow controllers <https://community.rti.com/forum-topic/transfering-large-data-over-dds>`_. It has slow, medium and fast flow controllers (seen in the Connext QoS profile link).

The medium flow controller produced the best results for our case.
However, the controllers will still need to be tuned for the particular machine/network/environment they are operating in.
The Connext flow controllers can be used to tune bandwidth and its aggressiveness for sending out data, though once the bandwidth of a particular setup is passed, performance will start to drop.
