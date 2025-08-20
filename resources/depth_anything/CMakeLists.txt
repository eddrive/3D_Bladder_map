cmake_minimum_required(VERSION 3.8)
project(depth_anything)  # CAMBIATO: nome package per Depth-Anything

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python executables - AGGIORNATI per Depth-Anything V2
install(PROGRAMS                 
  depth_anything/depth_anything.py    
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files - AGGIORNATI
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# NUOVO: Install config files (RViz configs, etc.)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL  # Opzionale se non hai ancora config files
)

# Install Python modules directory - AGGIORNATO per nuovo nome
install(DIRECTORY
  depth_anything
  DESTINATION lib/python3.10/site-packages
  FILES_MATCHING PATTERN "*.py"
)

# NUOVO: Install checkpoints directory per Depth-Anything V2 models (opzionale)
install(DIRECTORY
  checkpoints
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL  # Opzionale se tieni i modelli altrove
)

# NUOVO: Install masks directory (per endoscope masks)
install(DIRECTORY
  masks
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL  # Opzionale se le masks sono in /root/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()