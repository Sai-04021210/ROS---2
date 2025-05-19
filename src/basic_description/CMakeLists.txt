cmake_minimum_required(VERSION 3.8)
project(basic_description)

# Find ament macros and libraries
find_package(ament_cmake REQUIRED)

# Install URDF files
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install mesh files
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# Declare the package
ament_package()
