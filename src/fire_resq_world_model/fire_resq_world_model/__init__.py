"""World model - the single owner of believed state.

    Responsibility (Architecture.md section 3, Implementation_Plan.md section 9):
        detections -> association -> entity registry -> WorldState

    No other package writes world state. Cognition reads it; planning reads it; only this package updates it.

    Layout:
        config.py          WorldModelConfig (gates, smoothing, decay, safe zone), validated
        entities.py        status constants (mirroring VictimState.msg), tracks, the status lifecycle table
        world_model.py     WorldModel: observe() / set_status() / snapshot(). Pure Python, explicit time, no ROS.
        markers.py         what to draw for a snapshot (pure descriptions; the node turns them into RViz markers)
        world_model_node.py  the ROS glue: /fire_resq/detections -> WorldModel -> /fire_resq/world_state

    Independent of Gazebo, hardware and the camera: it sees only `DetectionArray` (camera-agnostic: it never looks at
    which backend produced a position), TF and its own parameters. It does not read the scenario.

    Not here on purpose (explicit TODOs, attached where they would go):
    TODO(future): Bayesian belief updates replacing the EMA smoothing and the "latest confidence" rule.
    TODO(future): uncertainty/covariance tracking per entity (Detection has no covariance yet).
    TODO(future): negative evidence ("looked there, saw nothing") - a victim inside the field of view that is not
                  seen does not lose confidence faster than one that is out of view.
    TODO(future): multiple fires, and obstacle memory beyond the SLAM grid.
    TODO(phase-9): a CARRIED victim's position should follow the robot; today its track is frozen.
    TODO(future): battery/resource state and dynamic fire risk belong to cognition, not here.
"""
