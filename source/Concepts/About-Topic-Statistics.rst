.. redirect-from::

    About-Topic-Statistics

About Topic Statistics
=================================


Overview
--------

ROS2 provides the integrated measurement of statistics for messages received by any
subscription. Allowing a user to collect subscription statistics enables them to characterize
the performance of their system or aid in diagnosis of any present issues.

The measurements provided are the received message age and received message period.
For each measurement the statistics calculated in a window and are the average, maximum, minimum,
standard deviation, and sample count.


How Statistics Are Calculated
-----------------------------

Each statistic set is calculated in constant time and constant memory
by using the utilities implemented in the
`libstatistics_collector <https://github.com/ros-tooling/libstatistics_collector>`__
package. When a new message is received by a subscription, this is a new sample for calculation in
the current measurement window. The average calculated is simply a
`moving average <https://en.wikipedia.org/wiki/Moving_average>`__. The maximum, minimum,
and sample count are updated upon receipt of each new sample, whereas the standard deviation is
calculated using `Welford's online algorithm <https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance\#Welford's_online_algorithm>`__.


Types of Statistics Calculated
------------------------------

* Received Message Period

  * units: milliseconds
  * uses the system clock to measure the period between received messages

* Received Message Age

  * units: milliseconds
  * requires a message to have a timestamp populated in the header field in order to calculate the age of the message as sent from a publisher.

Behavior
--------

By default, Topic Statistics measurements are not enabled. After enabling this feature for
a specific node via the subscription configuration options, both received message age and received
message period measurements are enabled for that specific subscription. The data is published using the
`MetricsMessage definition <https://github.com/ros2/rcl_interfaces/blob/master/statistics_msgs/msg/MetricsMessage.msg>`__
found in the `rcl_interfaces package <https://github.com/ros2/rcl_interfaces/tree/master/statistics_msgs>`__
at a configurable period (default 1 second) to a configurable topic (default /statistics). Here
note that the publishing period also serves as the sample collection window period.

Since received message period requires a message timestamp in a header field, empty data is published
if no timestamp is found. This behavior avoids the absence of a signal problem and is meant to explicitly
show that a measurement could not be made.

The first sample of each window for the received message period statistic does not yield a measurement.
This is because, in order to calculate this statistic, it requires knowing the time the previous
message arrived, so subsequent samples in the window yield measurements.

Comparison to ROS1
------------------

Similar to ROS1 `Topic Statistics <http://wiki.ros.org/Topics#Topic_statistics>`__, both message age
and message period are calculated, albeit from the subscription side. Other ROS1 metrics, e.g.,
the number of dropped messages or traffic volume, are currently not provided.

Support
-------

This feature is supported in ROS2 Foxy for C++ only (rclcpp).

