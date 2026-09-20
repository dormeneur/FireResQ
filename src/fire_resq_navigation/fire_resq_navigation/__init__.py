"""Navigation layer: depth -> /scan, SLAM / localization, Nav2.

Interfaces this layer exposes (stable between simulation and hardware):
    in : /camera/depth/image_raw + camera_info, /odom, TF (odom -> base_link -> camera frames)
    out: /scan, /map, TF map -> odom, Nav2 actions (/navigate_to_pose, ...), costmaps

Nothing here knows about Gazebo, and nothing above it needs to know which mapping/localization
backend produced map -> odom.

TODO(future): sensor noise on the simulated depth/odometry so SLAM quality is measured under
              realistic error (Phase 4 numbers are on noise-free odometry).
TODO(future): RGB-only visual SLAM alternative for the RGB-only camera option.
TODO(hardware): RealSense integration and validation of the depth-derived scan on the real device.
TODO(future): better loop closure - it made every configuration worse on a narrow-FOV scan.
TODO(future): dynamic environments (moving obstacles, changing maps).
"""
