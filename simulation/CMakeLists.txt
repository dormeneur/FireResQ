cmake_minimum_required(VERSION 3.8)
project(fire_resq_simulation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Scenario loading / validation / world generation (pure Python).
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/generate_world.py DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY urdf worlds models config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
