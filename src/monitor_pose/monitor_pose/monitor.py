#!/usr/bin/env python3
"""ROS 2 node that monitors rod payload pose and target setpoints.

Subscribes to:
- /model/rod_payload/pose         (geometry_msgs/msg/PoseStamped)
- /rod_target                     (std_msgs/msg/Float32MultiArray)

Publishes nothing.

Features:
- Real-time plotting of x, y, z, yaw, pitch for current pose and target
- CSV logging
- Safe quaternion -> yaw/pitch extraction

Expected target format:
  Float32MultiArray.data = [x, y, z, yaw, pitch]
with angles in radians.

Notes:
- Current pose orientation is converted from quaternion to yaw/pitch.
- Pitch is extracted from standard ZYX Euler angles.
- Roll is ignored by request.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray


@dataclass
class PoseSample:
    t: float
    x: float
    y: float
    z: float
    yaw: float
    pitch: float


@dataclass
class TargetSample:
    t: float
    x: float
    y: float
    z: float
    yaw: float
    pitch: float


class RodPoseMonitor(Node):
    def __init__(self):
        super().__init__('rod_pose_monitor')

        # Parameters
        self.declare_parameter('history_size', 1000)
        self.declare_parameter('plot_interval_ms', 50)
        self.declare_parameter('time_window_sec', 30.0)

        self.history_size = int(self.get_parameter('history_size').value)
        self.plot_interval_ms = int(self.get_parameter('plot_interval_ms').value)
        self.time_window_sec = float(self.get_parameter('time_window_sec').value)

        # Latest samples
        self._lock = threading.Lock()
        self.latest_current: Optional[PoseSample] = None
        self.latest_target: Optional[TargetSample] = None

        # Time series history for plotting
        self.t0 = time.time()
        self.history_t: Deque[float] = deque(maxlen=self.history_size)
        self.current_hist: dict[str, Deque[float]] = {
            'x': deque(maxlen=self.history_size),
            'y': deque(maxlen=self.history_size),
            'z': deque(maxlen=self.history_size),
            'yaw': deque(maxlen=self.history_size),
            'pitch': deque(maxlen=self.history_size),
        }
        self.target_hist: dict[str, Deque[float]] = {
            'x': deque(maxlen=self.history_size),
            'y': deque(maxlen=self.history_size),
            'z': deque(maxlen=self.history_size),
            'yaw': deque(maxlen=self.history_size),
            'pitch': deque(maxlen=self.history_size),
        }

        # ROS subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/model/rod_payload/pose',
            self.pose_callback,
            10,
        )
        self.target_sub = self.create_subscription(
            Float32MultiArray,
            '/rod_target',
            self.target_callback,
            10,
        )

        self.get_logger().info('Subscribed to /model/rod_payload/pose and /rod_target')

    @staticmethod
    def quaternion_to_yaw_pitch(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float]:
        """Convert quaternion to yaw and pitch using standard ZYX Euler convention.

        yaw   = rotation around Z
        pitch = rotation around Y
        roll  = rotation around X (ignored)
        """
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        return yaw, pitch

    def _append_history(self, t_rel: float, current: Optional[PoseSample], target: Optional[TargetSample]) -> None:
        self.history_t.append(t_rel)

        if current is not None:
            self.current_hist['x'].append(current.x)
            self.current_hist['y'].append(current.y)
            self.current_hist['z'].append(current.z)
            self.current_hist['yaw'].append(current.yaw)
            self.current_hist['pitch'].append(current.pitch)
        else:
            for key in self.current_hist:
                self.current_hist[key].append(float('nan'))

        if target is not None:
            self.target_hist['x'].append(target.x)
            self.target_hist['y'].append(target.y)
            self.target_hist['z'].append(target.z)
            self.target_hist['yaw'].append(target.yaw)
            self.target_hist['pitch'].append(target.pitch)
        else:
            for key in self.target_hist:
                self.target_hist[key].append(float('nan'))

    def pose_callback(self, msg: PoseStamped) -> None:
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z) + 0.5 
        qx = float(msg.pose.orientation.x)
        qy = float(msg.pose.orientation.y)
        qz = float(msg.pose.orientation.z)
        qw = float(msg.pose.orientation.w)
        yaw, pitch = self.quaternion_to_yaw_pitch(qx, qy, qz, qw)

        sample = PoseSample(time.time() - self.t0, x, y, z, yaw, pitch)

        with self._lock:
            self.latest_current = sample
            self._append_history(sample.t, self.latest_current, self.latest_target)

    def target_callback(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)
        if len(data) < 5:
            self.get_logger().warn(
                f'/rod_target expected 5 values [x, y, z, yaw, pitch], got {len(data)}: {data}'
            )
            return

        x, y, z, yaw, pitch = map(float, data[:5])
        sample = TargetSample(time.time() - self.t0, x, y, z, yaw, pitch)

        with self._lock:
            self.latest_target = sample
            self._append_history(sample.t, self.latest_current, self.latest_target)

    def shutdown(self) -> None:
        pass


def main(args=None):
    rclpy.init(args=args)
    node = RodPoseMonitor()

    # Spin ROS callbacks continuously so plotting does not throttle message handling.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        plt.style.use('seaborn-v0_8-whitegrid')
    except OSError:
        # Fall back to default style if seaborn style is unavailable.
        pass

    plt.ion()
    fig, axes = plt.subplots(5, 1, figsize=(12, 10), sharex=True)
    fig.subplots_adjust(left=0.08, right=0.98, top=0.93, bottom=0.08, hspace=0.14)
    fig.patch.set_facecolor('#f7f9fc')
    fig.suptitle('Rod Payload Pose: Current vs Target', fontsize=16, fontweight='semibold')

    labels = ['x', 'y', 'z', 'yaw', 'pitch']
    units = {'x': 'm', 'y': 'm', 'z': 'm', 'yaw': 'rad', 'pitch': 'rad'}
    y_limits = {
        'x': (-1.0, 4.0),
        'y': (-1.0, 4.0),
        'z': (-1.0, 4.0),
        'yaw': (-1.57, 1.57),
        'pitch': (-1.57, 1.57),
    }
    lines = {}
    target_lines = {}
    current_color = '#1f77b4'
    target_color = '#d62728'

    for ax, label in zip(axes, labels):
        ax.set_facecolor('#ffffff')
        ax.set_ylabel(f'{label} [{units[label]}]', fontsize=10)
        ax.set_ylim(*y_limits[label])
        line_current, = ax.plot(
            [], [],
            label='current',
            color=current_color,
            linewidth=2.0,
            alpha=0.95,
        )
        line_target, = ax.plot(
            [], [],
            label='target',
            color=target_color,
            linewidth=1.8,
            linestyle='--',
            alpha=0.95,
        )
        ax.legend(loc='upper right')
        ax.grid(True, which='major', linestyle='-', linewidth=0.7, alpha=0.35)
        ax.grid(True, which='minor', linestyle=':', linewidth=0.5, alpha=0.25)
        ax.minorticks_on()
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        lines[label] = line_current
        target_lines[label] = line_target
    axes[-1].set_xlabel('Time [s]', fontsize=11)

    def update(_frame):
        with node._lock:
            t = list(node.history_t)
            curr = {k: list(v) for k, v in node.current_hist.items()}
            targ = {k: list(v) for k, v in node.target_hist.items()}

        if not t:
            return list(lines.values()) + list(target_lines.values())

        for idx, label in enumerate(labels):
            lines[label].set_data(t, curr[label])
            target_lines[label].set_data(t, targ[label])
            axes[idx].set_ylim(*y_limits[label])

        # Prevent empty space when requested window is larger than retained history.
        x_left = max(t[0], t[-1] - node.time_window_sec)
        axes[-1].set_xlim(x_left, t[-1] + 0.1)
        fig.canvas.draw_idle()
        return list(lines.values()) + list(target_lines.values())

    ani = FuncAnimation(fig, update, interval=node.plot_interval_ms, blit=False)

    try:
        plt.show(block=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
