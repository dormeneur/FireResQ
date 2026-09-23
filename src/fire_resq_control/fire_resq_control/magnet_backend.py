"""The Phase 9 magnet abstraction: attach()/detach()/state, so magnet_node (and, later, the
rescue FSM) never knows whether ATTACH means a Gazebo detachable joint or a real MOSFET
switching a coil.

WHERE THE SIM/HARDWARE SPLIT ACTUALLY LIVES. Not here, and not as a second Python class. The one
backend below, RosTopicMagnetBackend, is itself fully hardware-neutral: it commands and reads
back exactly two standard ROS topics (`/fire_resq/magnet/energize`, `/fire_resq/magnet/state`) -
the same shape cmd_vel/odom already use (see simulation/urdf/gz_drive.xacro's own docstring) to
make sim<->hardware a driver swap, not a code change here. The Gazebo-specific driver on the
other end of those topics is simulation/scripts/sim_magnet_bridge.py (fire_resq_simulation, kept
out of this package on purpose - Gazebo-specific code does not belong in a package hardware
reuses). Its future hardware counterpart is fire_resq_hardware's ESP32 bridge (Phase 12,
TODO(phase-12) - not implemented, not pretended: no fake hardware behaviour lives here).

The client runs on its OWN private node/thread, the same reason Nav2PathQuery does
(fire_resq_cognition.nav2_path_query): so a blocking attach()/detach() call never depends on the
calling node's own executor also being free to run the subscription callback.
"""
from __future__ import annotations

import threading
import time
from abc import ABC, abstractmethod
from typing import Optional

from fire_resq_interfaces.msg import MagnetState
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool

_LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class MagnetBackend(ABC):
    """attach()/detach() block the CALLING thread until the driver reports an outcome (or
    `timeout_s` passes) and return the MagnetState that outcome was read from; `state` is the
    latest known state without waiting. Neither call raises on a failed attach/detach - a failure
    is a normal MagnetState with `attached` not matching what was requested, exactly so "requested"
    is never confused with "succeeded" (see magnet_node.py)."""

    @abstractmethod
    def attach(self, timeout_s: float) -> Optional[MagnetState]: ...

    @abstractmethod
    def detach(self, timeout_s: float) -> Optional[MagnetState]: ...

    @property
    @abstractmethod
    def state(self) -> Optional[MagnetState]: ...

    def destroy(self) -> None:
        ...


class RosTopicMagnetBackend(MagnetBackend):
    def __init__(self, energize_topic: str = '/fire_resq/magnet/energize',
                 state_topic: str = '/fire_resq/magnet/state'):
        self._node = Node('magnet_backend_client')
        self._pub = self._node.create_publisher(Bool, energize_topic, 10)
        self._lock = threading.Lock()
        self._state: Optional[MagnetState] = None
        self._updated = threading.Event()
        self._node.create_subscription(MagnetState, state_topic, self._on_state, _LATCHED)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception:                                   # the context was shut down under us
            pass

    def _on_state(self, msg: MagnetState) -> None:
        with self._lock:
            self._state = msg
        self._updated.set()

    @property
    def state(self) -> Optional[MagnetState]:
        with self._lock:
            return self._state

    def attach(self, timeout_s: float) -> Optional[MagnetState]:
        return self._command(True, timeout_s)

    def detach(self, timeout_s: float) -> Optional[MagnetState]:
        return self._command(False, timeout_s)

    def _command(self, energize: bool, timeout_s: float) -> Optional[MagnetState]:
        msg = Bool()
        msg.data = energize
        self._updated.clear()
        self._pub.publish(msg)
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0 or not self._updated.wait(remaining):
                return self.state                             # timed out: whatever we last heard, if anything
            self._updated.clear()
            s = self.state
            if s is not None and s.energized == energize:      # the driver's answer to THIS command
                return s
            # a stale/unrelated update (e.g. the driver's own startup message) - keep waiting

    def destroy(self) -> None:
        self._executor.shutdown()
        self._thread.join(timeout=2.0)
        self._node.destroy_node()
