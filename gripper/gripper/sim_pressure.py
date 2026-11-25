#!/usr/bin/env python3
"""
Sensor geometry (upside-down equilateral triangle, viewed from below):

     B
    (180°)
      |
      O
     / \
    C   A
 (270°)(60°)

  All three cups are SENSOR_RADIUS metres from the gripper TF origin,
  equally spaced at 120° with the flat edge at the top (C pointing down).


Usage
-----
  # 1. Set apple position as node parameters, then run:
  ros2 run fake_pressure fake_pressure_node.py \\
      --ros-args \\
      -p gripper_frame:=tool0 \\
      -p world_frame:=world \\
      -p apple_x:=0.5 \\
      -p apple_y:=0.0 \\
      -p apple_z:=0.3

  # 2. Trigger publishing from a second terminal:
  ros2 service call /fake_pressure/start std_srvs/srv/Trigger {}
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Int16MultiArray, MultiArrayDimension, MultiArrayLayout
from std_srvs.srv import Trigger

from tf2_ros import (Buffer,TransformListener,LookupException,ExtrapolationException,ConnectivityException)

_rng_a = random.Random(11)
_rng_b = random.Random(22)
_rng_c = random.Random(33)

def compute_pressure(dist_to_surface: float,
                     approach_radius: float,
                     contact_depth_scale: float,
                     pressure_far: float,
                     pressure_contact: float,
                     noise_amplitude: float,
                     rng: random.Random) -> int:
    """
    Return a simulated pressure (mmHg) as int16 for a single sensor given its
    signed distance to the apple surface (negative = penetrating).

    Pressure curve
    ──────────────
    idle  (d ≥ approach_radius) : pressure_far
    approach (0 < d < approach_r): cubic ease-in toward pressure_contact
    contact onset (d = 0)        : ~90 % drop (clamped)
    deep contact (d < 0)         : exponential saturation → pressure_contact
    """
    pressure_range = pressure_far - pressure_contact
    contact_depth  = max(0.0, -dist_to_surface)
    in_contact     = dist_to_surface <= 0.0

    if not in_contact:
        if dist_to_surface >= approach_radius:
            drop_fraction = 0.0
        else:
            t = dist_to_surface / approach_radius   # 1 at edge, 0 at surface
            drop_fraction = (1.0 - t) ** 3
    else:
        k = math.log(9.0) / contact_depth_scale
        drop_fraction = 1.0 - math.exp(-k * contact_depth)
        drop_fraction = max(0.9, min(1.0, drop_fraction))
        if contact_depth == 0.0:
            drop_fraction = 0.90

    raw = pressure_far - pressure_range * drop_fraction
    raw = max(pressure_contact, min(pressure_far, raw))

    noisy = raw + rng.gauss(0.0, noise_amplitude)
    noisy = max(pressure_contact, min(pressure_far, noisy))

    return int(round(noisy))


class FakePressureNode(Node):
    """
    Waits for a /fake_pressure/start service call, then publishes
    [pressure_a, pressure_b, pressure_c, tof_mm] at 10 Hz on
    MicroROS/sensor_data until the node is killed.
    """

    def __init__(self):
        super().__init__('fake_pressure_node')

        self.declare_parameter('gripper_frame',       'tool0')
        self.declare_parameter('world_frame',         'world')
        self.declare_parameter('apple_x',             0.5)
        self.declare_parameter('apple_y',             0.0)
        self.declare_parameter('apple_z',             0.3)
        self.declare_parameter('apple_radius',        0.04)    # 80 mm
        self.declare_parameter('sensor_radius',       0.20)    # 20 cm triangle
        self.declare_parameter('approach_radius',     0.005)   # 5 mm sigmoid trigger
        self.declare_parameter('contact_depth_scale', 0.015)
        self.declare_parameter('noise_amplitude',     2.0)
        self.declare_parameter('pressure_far',        1000.0)
        self.declare_parameter('pressure_contact',    400.0)
        self.declare_parameter('publish_rate_hz',     10.0)

        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(Int16MultiArray,'microROS/sensor_data',10)

        self._running = False
        self._timer   = None
        self._srv = self.create_service(Trigger,'fake_pressure/start',self._handle_start)

        self.get_logger().info(
            'fake_pressure_node ready.\n'
            '  Call  ros2 service call /fake_pressure/start std_srvs/srv/Trigger {}\n'
            '  to begin publishing on MicroROS/sensor_data at '
            f'{self.get_parameter("publish_rate_hz").value} Hz.'
        )

    def _handle_start(self, _request, response):
        if self._running:
            response.success = True
            response.message = 'Already publishing — no change.'
            return response

        rate = self.get_parameter('publish_rate_hz').value
        self._timer   = self.create_timer(1.0 / rate, self._publish)
        self._running = True

        apple_x = self.get_parameter('apple_x').value
        apple_y = self.get_parameter('apple_y').value
        apple_z = self.get_parameter('apple_z').value
        r = self.get_parameter('apple_radius').value

        response.success = True
        response.message = (
            f'Publishing started at {rate} Hz.  '
            f'Apple centre ({apple_x}, {apple_y}, {apple_z})  '
            f'radius {r*1000:.0f} mm.'
        )
        self.get_logger().info(response.message)
        return response

    def _publish(self):
        gripper_frame       = self.get_parameter('gripper_frame').value
        world_frame         = self.get_parameter('world_frame').value
        apple_x             = self.get_parameter('apple_x').value
        apple_y             = self.get_parameter('apple_y').value
        apple_z             = self.get_parameter('apple_z').value
        apple_radius        = self.get_parameter('apple_radius').value
        sensor_radius       = self.get_parameter('sensor_radius').value
        approach_radius     = self.get_parameter('approach_radius').value
        contact_depth_scale = self.get_parameter('contact_depth_scale').value
        noise_amplitude     = self.get_parameter('noise_amplitude').value
        pressure_far        = self.get_parameter('pressure_far').value
        pressure_contact    = self.get_parameter('pressure_contact').value

        try:
            tf = self._tf_buffer.lookup_transform(
                world_frame,
                gripper_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except (LookupException, ExtrapolationException, ConnectivityException) as exc:
            self.get_logger().warn(
                f'TF lookup failed: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        gx = tf.transform.translation.x
        gy = tf.transform.translation.y
        gz = tf.transform.translation.z

        # ── Time of flight (gripper centre → apple centre, in mm) ─────────
        dx = gx - apple_x
        dy = gy - apple_y
        dz = gz - apple_z
        tof_m  = math.sqrt(dx*dx + dy*dy + dz*dz)
        tof_mm = int(round(tof_m * 1000.0))

        angles_deg = [60.0, 180.0, 270.0]   # a:60, b:180, c:270 from +X axis
        sensor_names = ['a', 'b', 'c']
        rngs         = [_rng_a, _rng_b, _rng_c]
        pressures    = []
 
        for angle_deg, name, rng in zip(angles_deg, sensor_names, rngs):
            angle_rad = math.radians(angle_deg)
            sx = gx + sensor_radius * math.cos(angle_rad)   # X = down
            sy = gy + sensor_radius * math.sin(angle_rad)   # Y = right
            sz = gz                                          # Z = approach axis, no offset
 
            # Distance from this sensor to the apple centre
            ddx = sx - apple_x
            ddy = sy - apple_y
            ddz = sz - apple_z
            dist_to_centre  = math.sqrt(ddx*ddx + ddy*ddy + ddz*ddz)
            dist_to_surface = dist_to_centre - apple_radius
 
            p = compute_pressure(dist_to_surface,approach_radius,contact_depth_scale,pressure_far,pressure_contact,noise_amplitude,rng)
            pressures.append(p)
 
        # Layout: [pressure_a, pressure_b, pressure_c, tof_mm]
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout()
        msg.layout.dim = [
            MultiArrayDimension(
                label='sensor_data',
                size=4,
                stride=4,
            )
        ]
        msg.layout.data_offset = 0
        msg.data = [pressures[0], pressures[1], pressures[2], tof_mm]
 
        self._pub.publish(msg)
 
        self.get_logger().info(f'[pa={pressures[0]:4d}  pb={pressures[1]:4d}  'f'pc={pressures[2]:4d}  tof={tof_mm:5d} mm]',throttle_duration_sec=0.5,)
 
def main(args=None):
    rclpy.init(args=args)
    node = FakePressureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()