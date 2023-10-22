cmake_minimum_required(VERSION 3.8)
project(ur_robotiq_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY urdf meshes 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
