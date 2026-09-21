"""The path-query adapter: answers a PlanningContext path query with Nav2's ComputePathToPose.

The action client lives on its OWN private node, spun by its own thread. That is what lets the cognition node stay a plain
single-threaded node (a MultiThreadedExecutor cost ~0.9 cores under a 250 Hz simulated clock, measured) while a decision
blocks waiting for the planner: the planner's replies are handled on the private thread, never on the blocked one.

This is the ONLY place cognition touches navigation, and it only ASKS: it sends a start and a goal to the planner server
and reads back the verdict and the path length. It plans nothing itself. Everything else in the package sees just the
callable `(start_xy, goal_xy) -> PathInfo`.

Classification is deliberately conservative. `unreachable` is reserved for the planner saying so (no path, goal occupied
or outside the map). Anything that means "could not ask" - server missing, timeout, TF error - is `unknown`, which the
prioritizers treat as NOT reachable: a broken planner must never turn into an optimistic selection.
"""
from __future__ import annotations

import math
import threading
from typing import Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from .types import XY, PathInfo

R = ComputePathToPose.Result
_UNREACHABLE = {R.GOAL_OCCUPIED: 'goal occupied', R.GOAL_OUTSIDE_MAP: 'goal outside the map',
                R.NO_VALID_PATH: 'no valid path'}
_UNKNOWN = {R.UNKNOWN: 'planner error', R.INVALID_PLANNER: 'invalid planner', R.TF_ERROR: 'TF error',
            R.START_OUTSIDE_MAP: 'start outside the map', R.START_OCCUPIED: 'start occupied', R.TIMEOUT: 'planner timeout'}


class Nav2PathQuery:
    """Callable path query over the `compute_path_to_pose` action. Blocks the CALLING thread (on events) until the planner
    answers or `timeout_s` passes; the planner's replies are handled on this adapter's own thread, so it is safe to call
    from a callback of a single-threaded node. Uses wall time (the private node needs no simulated clock)."""

    def __init__(self, action_name: str = '/compute_path_to_pose', frame: str = 'map', planner_id: str = '',
                 timeout_s: float = 3.0):
        self.frame, self.planner_id, self.timeout_s = frame, planner_id, timeout_s
        self.node = Node('prioritizer_planner_client')
        self.client = ActionClient(self.node, ComputePathToPose, action_name)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()
        self.queries = 0

    def _spin(self) -> None:
        try:
            self.executor.spin()
        except Exception:                                               # the context was shut down under us
            pass

    def _pose(self, xy: XY) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = self.frame
        p.pose.position.x, p.pose.position.y = float(xy[0]), float(xy[1])
        p.pose.orientation.w = 1.0
        return p

    def __call__(self, start: XY, goal: XY) -> PathInfo:
        self.queries += 1
        if not self.client.wait_for_server(timeout_sec=0.5):
            return PathInfo.unknown('planner action server not available')
        req = ComputePathToPose.Goal()
        req.goal, req.start, req.use_start, req.planner_id = self._pose(goal), self._pose(start), True, self.planner_id

        accepted, done = threading.Event(), threading.Event()
        box: dict = {}

        def on_goal(fut):
            box['handle'] = fut.result()
            accepted.set()
            if box['handle'].accepted:
                box['handle'].get_result_async().add_done_callback(on_result)
            else:
                done.set()

        def on_result(fut):
            box['result'] = fut.result()
            done.set()

        self.client.send_goal_async(req).add_done_callback(on_goal)
        if not accepted.wait(self.timeout_s):
            return PathInfo.unknown('planner did not accept the request in time')
        if not box['handle'].accepted:
            return PathInfo.unknown('planner rejected the request')
        if not done.wait(self.timeout_s):
            box['handle'].cancel_goal_async()
            return PathInfo.unknown('planner did not answer in time')
        return self._classify(box['result'])

    @staticmethod
    def _classify(res) -> PathInfo:
        result = res.result
        code = int(result.error_code)
        poses = result.path.poses
        if res.status == GoalStatus.STATUS_SUCCEEDED and code == R.NONE and len(poses) > 0:
            pts = [(p.pose.position.x, p.pose.position.y) for p in poses]
            return PathInfo.reachable(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])))
        why = result.error_msg or ''
        if code in _UNREACHABLE:
            return PathInfo.unreachable(f'{_UNREACHABLE[code]}{": " + why if why else ""}')
        if code in _UNKNOWN:
            return PathInfo.unknown(f'{_UNKNOWN[code]}{": " + why if why else ""}')
        if res.status == GoalStatus.STATUS_SUCCEEDED and code == R.NONE:
            return PathInfo.unreachable('the planner returned an empty path')
        return PathInfo.unknown(f'planner ended with status {res.status}, error {code}')

    def destroy(self) -> Optional[None]:
        self.executor.shutdown()
        self.thread.join(timeout=2.0)
        self.client.destroy()
        self.node.destroy_node()
