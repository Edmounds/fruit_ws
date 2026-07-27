# pb_diff_pid_pursuit_controller

Nav2 path-following controller for a two-wheel differential base. It retains the original
lookahead, PID, approach, curvature and collision checks while producing only `linear.x` and
`angular.z`. It rotates in place for large heading errors and aligns final yaw near the goal.
