"""Planning - the rescue mission: perception -> world model -> cognition -> Nav2 -> ALIGN -> magnet -> safe zone -> reassess.

    Responsibility (Architecture.md section 3, Implementation_Plan.md sections 11 and 13 Phase 10):
        an explicit finite-state machine that carries ONE victim at a time from "cognition chose it" to "the world
        model says RESCUED", and re-runs cognition after every rescue and every failed attempt.

    Modules (the procedure is pure Python and testable without ROS or Gazebo; only rescue_node touches ROS):
        types.py      Observation / Event in, Effect out; the State enum
        config.py     every engineering constant, validated, each with its reason
        geometry.py   magnet contact, the carried victim's position, the approach pose, release spots in the safe zone
        coverage.py   SEARCH: which free cells a stationary look has reached, and where to look next (from the saved map)
        fsm.py        RescueFSM: the states, the declared transitions, ALIGN, ATTACH, VERIFY_*, failure handling
        rescue_node.py  ROS glue + /fire_resq/mission_state + the ExecuteRescue action

    The FSM holds exactly ONE victim id at a time and re-derives everything else. There is no waypoint list, no
    precomputed ordering and no stored path. It does not rank victims: SELECT_TARGET asks cognition.

    Implemented in Phase 10.

    TODO(future): recovery beyond retry-then-give-up (re-observation of a lost victim, negative evidence in the world model).
    TODO(future): closed-loop visual servoing for ALIGN inside the camera's near clip (open-loop on the stored position there).
    TODO(future): a CARRIED track that follows the robot in the world model (the FSM tracks the carried victim itself).
    TODO(future): decision hysteresis (an abandoned target is simply reconsidered; cognition may pick it again).
    TODO(hardware): the magnet's hold sensor (current sense / hall switch) behind MagnetState.attached; the cmd_vel watchdog.
"""
