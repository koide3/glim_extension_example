cmake_minimum_required(VERSION 3.16)
project(glim_extension_tutorial)

set(CMAKE_CXX_STANDARD 17)    # Enable C++17 

find_package(glim REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Odometry monitor example module
ament_auto_add_library(monitor_example SHARED
  src/monitor_example.cpp
)
target_link_libraries(monitor_example
  glim::glim
)

# Position constraint example module
ament_auto_add_library(position_constraint_example SHARED
  src/position_constraint_example.cpp
)
target_link_libraries(position_constraint_example
  glim::glim
)

ament_auto_package()
