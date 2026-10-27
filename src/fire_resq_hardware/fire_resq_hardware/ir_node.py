"""ROBOT SIDE (Raspberry Pi). Two digital IR obstacle sensors -> /fire_resq/ir_left, /fire_resq/ir_right
(std_msgs/Bool, True = something is close in front of that sensor).

Wiring (2026-10-16, from the maintainer):
    IR 1 (left)  OUT -> GPIO17, physical pin 11
    IR 2 (right) OUT -> GPIO27, physical pin 13
Both on gpiochip0, confirmed by line name ("GPIO17"/"GPIO27") on this Pi - no renumbering to work around.

These are the common 3-pin comparator obstacle sensor modules: OUT swings fully between the two rails, and the
module's own potentiometer sets a fixed (if adjustable) trigger range - there is no distance to read, only
"something is within range" or not. `active_low` (default True) picks which level means "something is there":
both pins read electrically HIGH with nothing in front of them (checked once on the bench, 2026-10-16), and
HIGH-when-clear/LOW-when-triggered is how nearly every one of these modules is wired. NOT verified against an
actual trigger or either module's datasheet - if "clear" shows as "object detected" with nothing in front of a
sensor, flip `active_low` in config/robot.yaml and redeploy.

Every reading is published, not just changes (`rate_hz`), so a consumer can tell "nothing is there" apart from
"the sensors stopped reporting" by how old the last message is - the same reason camera_node publishes
continuously rather than on motion.
"""
from __future__ import annotations

import threading

import gpiod
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class IrNode(Node):
    def __init__(self, **kwargs):
        super().__init__('ir_node', **kwargs)
        d = self.declare_parameter
        self.chip_path = d('chip', '/dev/gpiochip0').value
        self.left_line = int(d('left_line', 17).value)
        self.right_line = int(d('right_line', 27).value)
        self.active_low = bool(d('active_low', True).value)
        self.rate_hz = float(d('rate_hz', 20.0).value)
        self.debounce = max(1, int(d('debounce_samples', 2).value))     # agreeing raw reads before believing a change

        self.left_pub = self.create_publisher(Bool, d('left_topic', '/fire_resq/ir_left').value, 10)
        self.right_pub = self.create_publisher(Bool, d('right_topic', '/fire_resq/ir_right').value, 10)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _open(self):
        chip = gpiod.Chip(self.chip_path)
        lines = chip.get_lines([self.left_line, self.right_line])
        lines.request(consumer='fire_resq_ir', type=gpiod.LINE_REQ_DIR_IN)
        return chip, lines

    def _read_loop(self) -> None:
        chip, lines = None, None
        period = 1.0 / self.rate_hz
        believed = [False, False]                                 # the published state per side
        pending, agree = [None, None], [0, 0]                     # debounce: a change must repeat before it is believed
        while not self._stop.is_set():
            try:
                if lines is None:
                    chip, lines = self._open()
                    self.get_logger().info(f'{self.chip_path} lines {self.left_line},{self.right_line}: IR sensors connected')
                raw = lines.get_values()                          # [left, right], each 0 or 1, read together
                triggered = [(v == 0) if self.active_low else (v == 1) for v in raw]
                for i, side in enumerate(('left', 'right')):
                    if triggered[i] == believed[i]:
                        pending[i], agree[i] = None, 0
                        continue
                    if pending[i] != triggered[i]:
                        pending[i], agree[i] = triggered[i], 0
                    agree[i] += 1
                    if agree[i] >= self.debounce:
                        believed[i] = triggered[i]
                        self.get_logger().info(f'IR {side}: {"object detected" if believed[i] else "clear"}')
                for pub, value in zip((self.left_pub, self.right_pub), believed):
                    msg = Bool()
                    msg.data = value
                    pub.publish(msg)
                self._stop.wait(period)
            except Exception as e:                                # the sensor or the chip went away: retry, never crash
                if self._stop.is_set():
                    break
                self.get_logger().error(f'ir: {e}; retrying in 2 s', throttle_duration_sec=10.0)
                if lines is not None:
                    try:
                        lines.release()
                    except Exception:
                        pass
                lines, chip = None, None
                believed, pending, agree = [False, False], [None, None], [0, 0]
                self._stop.wait(2.0)
        if lines is not None:
            try:
                lines.release()
            except Exception:
                pass

    def destroy_node(self):
        self._stop.set()
        self._thread.join(timeout=3.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IrNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
