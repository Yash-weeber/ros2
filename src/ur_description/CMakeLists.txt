cmake_minimum_required(VERSION 3.8)
project(ur_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# 1. Install Python scripts to the lib directory
# This allows 'ros2 run ur_description test.py' to work
install(PROGRAMS
  scripts/test.py
  DESTINATION lib/${PROJECT_NAME}
)

# 2. Install resource directories to the share directory
# combined everything into ONE block to avoid conflicts
install(DIRECTORY
  config
  launch
  rviz
  urdf
  meshes
  data
  DESTINATION share/${PROJECT_NAME}
)
# Add this near the bottom, before ament_package()
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
