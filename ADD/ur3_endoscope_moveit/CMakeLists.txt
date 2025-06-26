cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(ur3_endoscope_moveit)

# Find ROS 2 standard packages
find_package(ament_cmake REQUIRED)
find_package(launch REQUIRED)
find_package(launch_ros REQUIRED)
find_package(ros_environment REQUIRED)

# Find MoveIt! related packages that your config will depend on
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_configs_utils REQUIRED) # Often used by launch files in MoveIt! config packages
find_package(rviz2 REQUIRED) # If you have rviz config files
find_package(xacro REQUIRED) # If your URDF/SRDF is using xacro
# Add your robot description package, which provides the base URDF
find_package(ur_description REQUIRED)
# Add the base ur_moveit_config if you are inheriting from it
find_package(ur_moveit_config REQUIRED)

# Check for testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install directories for launch files, config files, and SRDF
# The setup assistant will put files in these directories (config, launch, rviz, srdf).
# Ensure these directories exist in your package (even if empty initially).
install(DIRECTORY launch config rviz srdf
  DESTINATION share/${PROJECT_NAME}
  # This pattern ensures only files are installed, preserving directory structure but skipping empty ones
  FILE_PATTERN ".+" # This installs all files
  DIRECTORY_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

# Mark as an ament_package, which makes it visible to ROS 2
ament_package()