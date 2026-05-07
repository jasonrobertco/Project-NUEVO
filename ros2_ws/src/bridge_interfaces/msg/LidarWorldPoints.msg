std_msgs/Header header
float32[] xs        # World-frame X coordinates (mm), parallel with ys
float32[] ys        # World-frame Y coordinates (mm), parallel with xs
float32 robot_x     # Fused pose X at time of scan (mm)
float32 robot_y     # Fused pose Y at time of scan (mm)
float32 robot_theta # Fused pose heading at time of scan (radians)
