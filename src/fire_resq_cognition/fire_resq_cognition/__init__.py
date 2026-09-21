"""Cognition - reasoning and decision.

    Responsibility (Cognitive_Model.md, Implementation_Plan.md section 10):
        WorldView -> score every candidate victim -> select ONE, with a full explanation

    Layout:
        types.py         plain-data inputs and outputs (WorldView, PlanningContext, ScoredVictim, Decision). No ROS.
        config.py        PrioritizerConfig: weights, hazard model, normalisation, approach standoff. Validated.
        factors.py       the individual factor functions (fire risk, proximity, accessibility, normalisation)
        prioritizer.py   VictimPrioritizer ABC, WeightedUtilityPrioritizer (MVP), NearestVictimPrioritizer (baseline)
        nav2_path_query.py  the ROS adapter that answers the PlanningContext path query with Nav2's ComputePathToPose
        prioritizer_node.py the ROS glue: /fire_resq/world_state -> decision -> /fire_resq/rescue_target + SelectTarget

    Hard rules this package exists to enforce:
      * Rescue order is NEVER hardcoded. It is recomputed from the current world model, every time.
      * The decision model sits behind the VictimPrioritizer interface so it can be replaced without touching anything else.
      * Every decision carries its own explanation: each candidate's factors, weighted contributions and utility, and the
        reason any candidate was excluded. An unexplainable decision is a failed decision.
      * A candidate that cannot be reached is EXCLUDED and says so; it is never silently selected or silently dropped.

    Everything except the two ROS modules is pure Python (no rclpy), so the decision logic is unit-testable and cannot
    import Gazebo, hardware, a camera or detector internals (guarded by tests). Navigation is reached only through the
    path-query callable in PlanningContext: cognition never plans a path itself.

    Not here on purpose (explicit TODOs, attached where they would go):
    TODO(future): Bayesian / uncertainty-aware reasoning (confidence is a plain weighted term; no covariance is consumed).
    TODO(future): active perception and information-gain actions (a low-confidence victim is only deprioritised, never inspected).
    TODO(future): dynamic fire-risk modelling: fire_risk is a STATIC function of distance to the fire (see factors.fire_risk).
    TODO(future): battery/resource-aware decision-making (no energy term; rescue_cost is path length in metres only).
    TODO(future): decision hysteresis / commitment: a re-decision may switch target when scores are nearly tied.
    TODO(future): learned policy behind the same interface.
    TODO(phase-10): the executable approach pose (magnet standoff, yaw) is the rescue FSM's; this package only computes an
                    approach POINT to ask the planner about.
"""
