"""Cognition - reasoning and decision.

    Responsibility (Cognitive_Model.md, Implementation_Plan.md section 10):
        WorldState -> score every unrescued victim -> select ONE RescueTarget

    Hard rules this package exists to enforce:
      * Rescue order is NEVER hardcoded. It is recomputed from the current world model.
      * The decision model sits behind the VictimPrioritizer interface so it can be
        replaced without touching anything else.
      * Every decision emits a ScoreBreakdown and a rationale. An unexplainable
        decision is a failed decision.

    This package must not import Gazebo APIs, hardware modules, or a specific camera.

    Implemented in Phase 8 (after Nav2, which supplies path length and reachability).
    Empty by design until then.

    TODO(phase-8): VictimPrioritizer interface.
    TODO(phase-8): WeightedUtilityPrioritizer - transparent weighted utility over
                   fire risk, distance to fire, robot distance, accessibility,
                   rescue cost and perception confidence. Weights are ROS params.
    TODO(phase-8): NearestVictimPrioritizer - the PRD section 11 evaluation baseline.
    TODO(future): Bayesian / uncertainty-aware reasoning.
    TODO(future): active perception and information-gain actions.
    TODO(future): dynamic fire-risk modelling (risk is a static function for the MVP).
    TODO(future): battery/resource-aware decision-making.
    TODO(future): learned policy behind the same interface.
"""
