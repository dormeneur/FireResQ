"""ROS glue for the electromagnet: /fire_resq/set_magnet (SetMagnet) -> a backend -> MagnetState.

Thin on purpose, exactly like prioritizer_node/world_model_node: this node holds no attach/detach
logic of its own. It turns one SetMagnet request into one backend.attach()/detach() call, waits
for the backend's answer (bounded by a timeout parameter), and reports success ONLY if the
resulting state actually shows what was asked for - `success=True` never just means "the request
was accepted" (Hard rule: PICK_UP/RELEASE confirm via MagnetState, not the request itself; see
docs/Implementation_Plan.md Phase 9 and CLAUDE.md "State machine / safety").

It does not decide WHICH victim to pick up, does not navigate, and does not mark anything
TARGETED/CARRIED/RESCUED - that is the rescue FSM's (Phase 10). This node knows nothing about
Gazebo or GPIO; see magnet_backend.py for where the sim/hardware split actually lives.
"""
from __future__ import annotations

import rclpy
from fire_resq_interfaces.msg import MagnetState
from fire_resq_interfaces.srv import SetMagnet
from rclpy.node import Node

from .magnet_backend import RosTopicMagnetBackend


class MagnetNode(Node):
    def __init__(self, backend=None, **kwargs):
        """`backend` overrides the ROS-topic backend (tests inject a fake); by default it talks to
        whichever driver is running (the sim bridge today, the ESP32 bridge on hardware, Phase 12)."""
        super().__init__('magnet_node', **kwargs)
        for name, val in (('energize_topic', '/fire_resq/magnet/energize'),
                          ('state_topic', '/fire_resq/magnet/state'),
                          ('attach_timeout_s', 3.0), ('detach_timeout_s', 3.0),
                          ('set_magnet_service', '/fire_resq/set_magnet')):
            self.declare_parameter(name, val)
        p = self.get_parameter
        self.attach_timeout_s = float(p('attach_timeout_s').value)
        self.detach_timeout_s = float(p('detach_timeout_s').value)

        self.backend = backend or RosTopicMagnetBackend(p('energize_topic').value, p('state_topic').value)
        self.create_service(SetMagnet, p('set_magnet_service').value, self._on_set_magnet)
        self.get_logger().info('magnet_node ready')

    def _on_set_magnet(self, request: SetMagnet.Request, response: SetMagnet.Response) -> SetMagnet.Response:
        timeout = self.attach_timeout_s if request.attach else self.detach_timeout_s
        state = self.backend.attach(timeout) if request.attach else self.backend.detach(timeout)
        response.state = state if state is not None else MagnetState()
        # A request only succeeds if the reported state actually shows the requested outcome -
        # "requested" is never treated as "done" (see module docstring).
        response.success = bool(state is not None and state.attached == request.attach)
        if not response.success:
            reason = 'no MagnetState reply from the driver (is it running?)' if state is None else (
                'no victim within the magnet\'s reach' if request.attach else 'driver did not confirm release')
            self.get_logger().warn(f"SetMagnet(attach={request.attach}) failed: {reason}")
        return response

    def destroy_node(self):
        self.backend.destroy()
        super().destroy_node()


def main():
    rclpy.init()
    node = MagnetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
