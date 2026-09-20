"""Routes the TESTS ask the robot to drive (WORLD coordinates). The robot has no exploration
behaviour yet - where it goes is decided here, never by the robot."""

# Mapping lap: out along the south wall, up the east side, back along the north wall, then down the
# middle to the start. Every leg keeps >= 0.3 m from anything solid. ~14.8 m; translation legs joined
# by in-place turns, which is what the scan-matching SLAM preset handles well.
LAP_WORLD = [(0.7, -2.0), (2.0, -2.0), (2.0, 2.0), (-0.3, 2.0), (-0.3, -0.3), (-1.9, -1.9)]

# A different route, used to check localization against a saved map.
ROUTE_WORLD = [(-0.9, -0.9), (0.5, -0.2), (0.6, -2.0), (-0.5, -2.0), (-1.9, -1.9)]
