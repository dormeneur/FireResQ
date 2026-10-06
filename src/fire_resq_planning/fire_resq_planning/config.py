"""Planning parameters: one validated dataclass, so a bad value fails at start-up, not mid-mission.

Nothing here is a coordinate, a victim id or an order. The numbers are the rescue procedure's engineering
constants (speeds, tolerances, timeouts, retry budgets) and the small amount of geometry the FSM needs that the
robot description and the victim model already define (the magnet reach comes from the description at launch, the
ring radius is checked against the victim SDF by a unit test, the safe-zone size against the world model's config).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, fields
from typing import Any, Dict


@dataclass(frozen=True)
class PlanningConfig:
    # ---- robot / victim geometry (the description supplies magnet_reach_m at launch; see geometry.py) -------------
    magnet_reach_m: float = 0.075            # base_link -> the magnet's FRONT FACE: magnet_x + magnet_length / 2
    victim_ring_radius_m: float = 0.056      # the steel ring the magnet meets (checked against models/victim/model.sdf)
    safe_zone_size_x: float = 1.0            # the world model's configured safe zone (checked against its config)
    safe_zone_size_y: float = 1.0
    max_linear_mps: float = 0.30             # the robot's limits (the description supplies them at launch)
    max_angular_rps: float = 1.5

    # ---- approach (the contract cognition already fixes: 0.4 m short of the victim's centre, facing it) ---------------
    approach_standoff_m: float = 0.40
    nav_timeout_s: float = 150.0
    max_target_attempts: int = 2             # failures on one victim before it is given up as UNREACHABLE

    # ---- perceive: stationary look-around (positions are withheld while the camera turns, so turn, stop, look) -------
    sweep_max_step_rad: float = 1.05         # < the camera's FOV, so consecutive looks overlap
    sweep_turn_rps: float = 0.6
    sweep_dwell_s: float = 1.5
    sweep_yaw_tol_rad: float = 0.05
    camera_hfov_rad: float = 1.518           # the navigation launch's 87 degrees; replaced by CameraInfo when it arrives
    world_settle_s: float = 1.0              # UPDATE_WORLD: the world model must publish a belief newer than the last look

    # ---- align: the last stretch from the approach pose into magnet contact ----------------------------------------
    align_speed_mps: float = 0.06
    align_turn_rps: float = 0.5
    align_heading_tol_rad: float = 0.035
    align_heading_gain: float = 1.2
    align_stop_gap_m: float = -0.02          # stop when the magnet face is estimated this far PAST flush: pushing a 0.22 kg
    #                                          victim a couple of cm is harmless, stopping short of contact is not
    align_blind_center_dist_m: float = 0.30  # base_link -> victim centre below which the camera (0.10 m near clip) cannot see it
    align_lost_timeout_s: float = 1.5        # no victim in view for this long while it SHOULD be visible = lost
    align_bearing_gate_rad: float = 0.35     # a camera bearing further than this from the stored one is another object
    align_width_ratio: float = 0.5           # ...and a blob narrower than this fraction of a victim's width at the stored range is too
    victim_width_m: float = 0.10             # the victim's torso (models/victim/model.sdf: radius 0.05)
    align_max_extra_creep_m: float = 0.12    # bounded correction: never drive further than the contact distance + this
    align_max_start_error_m: float = 0.30    # the approach pose must be this close to the standoff, else Nav2 did not deliver
    align_timeout_s: float = 60.0

    # ---- attach / carry ---------------------------------------------------------------------------------------------
    attach_timeout_s: float = 10.0
    attach_settle_s: float = 1.0             # stand still in contact this long before energising (see ATTACH)
    attach_retries: int = 2                  # after a refusal: de-energise, creep a little further, try again
    attach_retry_creep_m: float = 0.02
    identity_gate_m: float = 0.25            # the belief nearest the expected victim position must be the target within this
    identity_clearance_m: float = 0.25       # ...and no OTHER belief may be this close to the magnet
    verify_carry_s: float = 1.5              # the hold must be reported continuously this long...
    tug_angle_rad: float = 0.20              # ...through a small turn (a weak hold does not survive motion)
    tug_turn_rps: float = 0.5
    depart_distance_m: float = 0.45           # after pick-up: turn away and drive this far (forward only) to leave the victim's old spot
    depart_timeout_s: float = 25.0
    depart_pause_s: float = 2.0               # ...and only after standing still this long
    depart_trigger_m: float = 0.15            # after a failed attempt, depart only from within this of touching the victim: where Nav2
                                             # cannot plan from (0.265 m of its centre, Phase 7/8)

    # ---- safe zone / release ----------------------------------------------------------------------------------------
    release_margin_m: float = 0.076          # the carried victim's centre must be this far inside the zone edge: its whole ring
                                             # (0.056) plus 2 cm. 0.10 refused a victim 8 cm inside - wholly in the zone (measured)
    release_spot_buffer_m: float = 0.075     # planned spots sit this much deeper again: Nav2 arrives within its goal tolerance
    release_spot_radius_m: float = 0.28      # release spots ring the zone centre (which the return-leg query needs free)
    release_spot_min_sep_m: float = 0.30     # between victims already released in the zone (~3 ring diameters)
    release_spot_count: int = 12              # per ring
    release_attempts: int = 3
    release_timeout_s: float = 10.0
    verify_release_s: float = 1.0
    costmap_clear_radius_m: float = 1.0      # forget obstacle marks this close to the robot when a victim joins or leaves it
    costmap_timeout_s: float = 5.0
    costmap_settle_s: float = 1.5            # the global costmap republishes at 1 Hz: let the cleared cells reach the planner

    # ---- world / selection ------------------------------------------------------------------------------------------
    min_target_confidence: float = 0.10      # a target whose belief decays below this has been lost sight of for too long...
    confidence_drop_ratio: float = 0.5       # ...AND below this fraction of its confidence when cognition chose it
    target_move_m: float = 0.35              # a target track that jumps this far is a different belief: reselect
    world_stale_s: float = 5.0
    select_timeout_s: float = 30.0
    select_retry_s: float = 3.0
    select_retries: int = 4                  # "no eligible victim" while known victims remain: retry, then search
    init_settle_s: float = 3.0
    init_timeout_s: float = 240.0
    mission_timeout_s: float = 2400.0

    # ---- search (coverage of the saved map; NOT a frontier explorer - the map already exists) ----------------------
    search_range_m: float = 4.0              # how far a colour blob still classifies reliably (V2 was found at 5 m)
    search_min_range_m: float = 1.0          # a blob nearer than this touches the image border and gets NO position: not a useful look
    coverage_cell_clearance_m: float = 0.30  # a victim's centre is at least this far from any wall/obstacle
    viewpoint_spacing_m: float = 0.50
    viewpoint_clearance_m: float = 0.40
    coverage_goal: float = 0.95
    min_gain_cells: int = 12                 # a viewpoint that would reveal fewer new cells is not worth the trip
    search_nav_timeout_s: float = 120.0
    max_viewpoint_failures: int = 6

    def __post_init__(self):
        positive = ('magnet_reach_m', 'victim_ring_radius_m', 'safe_zone_size_x', 'safe_zone_size_y', 'max_linear_mps',
                    'max_angular_rps', 'approach_standoff_m', 'align_speed_mps', 'align_turn_rps', 'sweep_turn_rps',
                    'sweep_max_step_rad', 'tug_turn_rps', 'nav_timeout_s', 'align_timeout_s', 'attach_timeout_s',
                    'mission_timeout_s', 'search_range_m', 'viewpoint_spacing_m')
        for k in positive:
            if not getattr(self, k) > 0:
                raise ValueError(f'{k} must be positive')
        if self.align_speed_mps > self.max_linear_mps or self.align_turn_rps > self.max_angular_rps \
                or self.sweep_turn_rps > self.max_angular_rps or self.tug_turn_rps > self.max_angular_rps:
            raise ValueError('a planning speed exceeds the robot\'s velocity limits')
        if self.approach_standoff_m <= self.contact_center_dist_m:
            raise ValueError('the approach standoff must leave a creep between it and magnet contact')
        if not 0 < self.coverage_goal <= 1:
            raise ValueError('coverage_goal must be in (0, 1]')
        if self.max_target_attempts < 1 or self.attach_retries < 0 or self.release_attempts < 1:
            raise ValueError('retry budgets must be non-negative (attempts >= 1)')
        if self.release_margin_m < self.victim_ring_radius_m:
            raise ValueError('release_margin_m must keep the whole victim ring inside the zone')
        if self.align_stop_gap_m > 0.0:
            raise ValueError('align_stop_gap_m must be <= 0: stopping short of flush contact cannot attach')

    @property
    def contact_center_dist_m(self) -> float:
        """base_link -> the victim's centre when the magnet face is exactly flush with the ring."""
        return self.magnet_reach_m + self.victim_ring_radius_m

    @classmethod
    def from_dict(cls, params: Dict[str, Any]) -> 'PlanningConfig':
        known = {f.name: f for f in fields(cls)}
        bad = sorted(k for k in params if k not in known)
        if bad:
            raise ValueError(f'unknown planning parameters: {bad}')
        out = {}
        for k, v in params.items():
            out[k] = int(v) if isinstance(known[k].default, int) and not isinstance(known[k].default, bool) else float(v)
        return cls(**out)

    def safe_zone_half_diag(self) -> float:
        return math.hypot(self.safe_zone_size_x, self.safe_zone_size_y) / 2
