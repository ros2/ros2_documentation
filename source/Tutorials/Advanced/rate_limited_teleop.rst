.. _rate_limited_teleop:

Smoothing keyboard teleop with a rate-limiter
=============================================

**Problem:** Direct keyboard teleoperation often produces jerky motion because
consecutive commands can jump from 0 → max in a single cycle.

**Goal:** Insert a small ROS 2 node between your keyboard teleop and the robot to
limit the per-cycle change in velocity commands (``/cmd_vel``), yielding smoother,
safer motion—especially in indoor labs and with differential drive bases.

When to use
-----------

- New drivers testing in small indoor spaces.
- Mobile robots where wheel slip or IMU spikes appear during sharp command changes.
- Teaching examples where safety and repeatability matter.

Topology
--------

.. code-block:: text

    teleop_twist_keyboard  ->  /cmd_vel_raw  ->  rate_limiter  ->  /cmd_vel

Reference node (Python)
-----------------------

The node below subscribes to ``/cmd_vel_raw`` (geometry_msgs/Twist), applies a
symmetric acceleration limit to linear and angular components, and publishes
smoothed commands to ``/cmd_vel`` at a fixed rate.

.. code-block:: python

    # twist_rate_limiter.py
    # Minimal ROS 2 node to rate-limit Twist commands.
    import math
    from dataclasses import dataclass
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist

    @dataclass
    class Ramp:
        rate_per_s: float
        dt: float
        def step(self, current: float, target: float) -> float:
            max_step = self.rate_per_s * self.dt
            delta = target - current
            if   delta >  max_step: return current + max_step
            elif delta < -max_step: return current - max_step
            else:                   return target

    class TwistRateLimiter(Node):
        def __init__(self):
            super().__init__('twist_rate_limiter')
            self.declare_parameters('', [
                ('in_topic',  'cmd_vel_raw'),
                ('out_topic', 'cmd_vel'),
                ('rate_hz',   50.0),
                ('lin_acc',   0.6),   # m/s^2
                ('ang_acc',   1.5),   # rad/s^2
            ])
            in_topic  = self.get_parameter('in_topic').get_parameter_value().string_value
            out_topic = self.get_parameter('out_topic').get_parameter_value().string_value
            self.dt   = 1.0 / self.get_parameter('rate_hz').value
            self.lin_ramp = Ramp(self.get_parameter('lin_acc').value, self.dt)
            self.ang_ramp = Ramp(self.get_parameter('ang_acc').value, self.dt)

            self.sub = self.create_subscription(Twist, in_topic, self._on_raw, 10)
            self.pub = self.create_publisher(Twist, out_topic, 10)
            self.timer = self.create_timer(self.dt, self._on_timer)

            self._target = Twist()
            self._current = Twist()

        def _on_raw(self, msg: Twist):
            self._target = msg

        def _on_timer(self):
            cur, tgt = self._current, self._target
            cur.linear.x  = self.lin_ramp.step(cur.linear.x,  tgt.linear.x)
            cur.linear.y  = self.lin_ramp.step(cur.linear.y,  tgt.linear.y)
            cur.linear.z  = self.lin_ramp.step(cur.linear.z,  tgt.linear.z)
            cur.angular.x = self.ang_ramp.step(cur.angular.x, tgt.angular.x)
            cur.angular.y = self.ang_ramp.step(cur.angular.y, tgt.angular.y)
            cur.angular.z = self.ang_ramp.step(cur.angular.z, tgt.angular.z)
            self.pub.publish(cur)

    def main():
        rclpy.init()
        node = TwistRateLimiter()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()

Usage
-----

1. Start your keyboard teleop to publish ``/cmd_vel_raw``:

   .. code-block:: bash

      ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_raw

2. Run the rate-limiter node:

   .. code-block:: bash

      ros2 run your_pkg twist_rate_limiter \
        --ros-args -p in_topic:=cmd_vel_raw -p out_topic:=cmd_vel \
                   -p rate_hz:=50.0 -p lin_acc:=0.6 -p ang_acc:=1.5

3. Drive as usual, but consume ``/cmd_vel`` on the robot controller.

Tuning tips
-----------

- **rate_hz:** 50 Hz is a good default for smoothness without excess CPU.
- **lin_acc / ang_acc:** Start with conservative values (0.4–0.8 m/s², 1.0–2.0 rad/s²),
  raise gradually until the robot feels responsive but not jerky.
- Keep the limiter running even when using joysticks; it protects against spiky inputs.

Testing snippet
---------------

.. code-block:: bash

    # Expect a gentle ramp up; no step jumps.
    ros2 topic pub -r 20 /cmd_vel_raw geometry_msgs/Twist "{linear: {x: 1.0}}"
    ros2 topic echo /cmd_vel

Notes
-----

This page adds a reusable pattern for safer teleoperation by inserting a simple
acceleration limiter; it’s intentionally minimal and can be dropped into any ROS 2 package.

See also
--------

If you use ``ros2_control``, consider controller-side rate limiting via
``control_toolbox::RateLimiter`` (C++).
It can constrain value, velocity (first derivative), and jerk (second derivative) directly in your controller update loop.
This keeps smoothing in the control stack rather than as a separate node from teleop.


- API: control_toolbox RateLimiter (rolling)
  https://docs.ros.org/en/rolling/p/control_toolbox/generated/classcontrol__toolbox_1_1RateLimiter.html
- Filter plugin variant: control_filters::RateLimiter
  https://control.ros.org/rolling/doc/api/classcontrol__filters_1_1RateLimiter.html
