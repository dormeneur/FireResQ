"""Python access to the robot description's numbers.

`parameters.xacro` is the single source of truth for the robot's geometry and limits. Anything that
needs a value from it at launch time (Nav2's footprint, perception's floor height) reads it HERE, so
no package restates a dimension. Pure Python; no ROS import.
"""
from .geometry import find_parameters_xacro, read_properties  # noqa: F401
