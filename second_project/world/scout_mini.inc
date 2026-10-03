# Scout Mini robot model for Stage simulator
# AgileX Scout Mini with mecanum wheels - omnidirectional drive

# Laser ranger sensor definition
define scout_laser ranger
(
  # Sensor specs matching the LiDAR used in Task 1
  sensor
  (
    range [0.1 6.0]      # Min/max range in meters (matches SLAM Toolbox config)
    fov 360.0            # Full 360-degree field of view
    samples 720          # 720 samples = 0.5 deg angular resolution
  )
  
  # Physical dimensions of the laser housing
  size [0.15 0.15 0.10]
  
  # Mounting position: centered on top of the robot at height 0.18 m
  pose [0.0 0.0 0.18 0.0]
  
  color "black"
)

# Main robot model definition
define scout_mini position
(
  # Real robot dimensions: 612 x 580 x 245 mm
  size [0.612 0.580 0.245]
  
  # Omnidirectional kinematics (mecanum wheels)
  drive "omni"
  
  # Velocity limits: max 2.8 km/h = 0.78 m/s
  velocity_bounds [-0.78 0.78 -0.78 0.78 0 0 -90 90]
  acceleration_bounds [-0.5 0.5 -0.5 0.5 0 0 -90 90]
  
  # Odometry with realistic noise (coherent with the ROS bag from Task 1)
  localization "odom"
  odom_error [0.05 0.05 0.10 0.05 0.05 0.20]
  
  # Center of rotation at geometric center
  origin [0.0 0.0 0.0 0.0]
  
  color "gray"
  gui_nose 1        # Show orientation arrow in Stage GUI
  ranger_return 1   # Robot is visible to other robots' rangefinders
  
  # Instantiate the laser sensor
  # stageros will publish the frame as base_laser_link automatically
  scout_laser (name "base_laser_link")
)