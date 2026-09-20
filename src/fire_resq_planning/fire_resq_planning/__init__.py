"""Planning - turns a chosen target into an executed rescue.

    Responsibility (Architecture.md section 3, Implementation_Plan.md section 11):
        SELECT VICTIM -> NAVIGATE -> ALIGN -> PICK UP -> RETURN
        -> RELEASE -> MARK RESCUED -> REASSESS

    The FSM holds exactly ONE victim id at a time and re-derives everything else.
    There is no waypoint list, no precomputed ordering and no stored path - the
    REASSESS -> SELECT VICTIM edge re-runs cognition against the updated world model.

    Implemented in Phase 10. Empty by design until then.

    TODO(phase-10): rescue FSM behind the ExecuteRescue action.
    TODO(phase-10): approach-pose computation (offset back along robot->victim bearing).
    TODO(future): recovery from failed navigation.
    TODO(future): recovery from failed pickup.
    TODO(future): recovery from failed detection (no target found).
    TODO(future): closed-loop visual servoing for ALIGN (open-loop in the MVP).
"""
