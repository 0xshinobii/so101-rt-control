# Per-iteration controller telemetry, published from the non-RT thread.
std_msgs/Header header
float64[3] ee_position     # end-effector world position [m]
float64[] joint_error      # q_des - q per joint [rad]
float64 arm_rms_error      # RMS of the arm-joint errors (joints 0..4) [rad]
float64 estimated_payload_mass # online estimate [kg], NaN when unavailable
float64[6] estimated_disturbance_torque # external joint-torque estimate [N.m]
