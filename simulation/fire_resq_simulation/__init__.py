"""FireResQ simulation support: scenario loading, validation and world generation.

Pure Python - no ROS, no Gazebo import - so it is fast to unit-test. The scenario YAML is the
ONLY place arena object placement lives (PRD/Architecture: no victim coordinates in cognition
or planning). Everything else consumes the generated world through perception.
"""

from .scenario import (  # noqa: F401
    Scenario, ScenarioError, build_world_sdf, load_scenario, resolve_scenario, write_world,
)
