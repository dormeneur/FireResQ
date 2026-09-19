# FireResQ — Cognitive Model

## Cognitive Objective
The robot must decide **which victim to rescue next** using information currently available. The rescue order must be dynamic, not fixed.

## World State
The cognitive layer reasons over:
```text
Robot
Fire
Victim A
Victim B
Victim C
Obstacles
Safe Zone
Resources
```

Entity state may include:
```text
position
confidence
status
risk
accessibility
```

## Initial Decision Model
For each unrescued victim, compute a transparent priority/utility score from factors such as:
- Fire risk.
- Distance from fire.
- Robot-to-victim distance.
- Accessibility.
- Estimated rescue cost.
- Perception confidence.

The exact formulation may evolve.

## Reassessment
```text
Observation
→ Update World State
→ Recalculate Priorities
→ Select Next Action
```

## Advanced TODOs
- TODO: probability-based beliefs.
- TODO: Bayesian updates.
- TODO: deliberate inspection of uncertain areas.
- TODO: battery/time as decision resources.
- TODO: compare weighted utility with probabilistic decision methods.
