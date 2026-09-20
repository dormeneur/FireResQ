"""World model - the single owner of believed state.

    Responsibility (Architecture.md section 3, Implementation_Plan.md section 9):
        detections -> association -> entity registry -> WorldState

    No other package writes world state. Cognition reads it; planning reads it;
    only this package updates it.

    Implemented in Phase 6. Empty by design until then.

    TODO(phase-6): entity registry with nearest-neighbour association and a gating radius.
    TODO(phase-6): EMA position smoothing and time-based confidence decay.
    TODO(phase-6): victim status lifecycle; RESCUED is terminal and ends candidacy.
    TODO(phase-6): RViz markers - this is how the system gets demonstrated.
    TODO(future): Bayesian belief updates replacing the EMA smoothing.
    TODO(future): uncertainty/covariance tracking per entity.
    TODO(future): negative evidence ("looked there, saw nothing").
"""
