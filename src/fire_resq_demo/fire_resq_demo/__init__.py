"""The bottle demo: the real robot finds coloured bottles with its one camera and visits each of them.

It is a DEMO, beside the rescue stack, not part of it: it does not use the world model, cognition, Nav2 or the
rescue FSM, because those need a `map` frame and this robot has nothing that can provide one (no encoders, no
depth camera; docs/Hardware.md section 5). What it shares with the rescue stack is the robot's drivers.

    vision.py       one frame -> bottles (by colour, and black ones by darkness) + where the floor stops
    seeker.py       the behaviour: remember bottles, nearest unvisited first, wait, avoid, finish  (pure)
    mapping.py      dead-reckoned pose and the sketch map drawn from it (NOT SLAM: read its docstring)
    dashboard.py    the annotated picture + the web page on http://localhost:8080 (index.html)
    demo_node.py    ROS plumbing: /camera/image_raw/compressed + /odom in, /cmd_vel out
    fake_world.py, fake_robot_node.py    a pretend room and robot, for rehearsal and tests

Run it with ./demo.sh from the repository root (docs/DEMO.md).

TODO(demo): a real localisation source would replace mapping.py's dead reckoning; nothing else here would change.
"""
