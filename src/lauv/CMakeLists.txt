cmake_minimum_required(VERSION 3.8)
project(lauv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  urdf
  meshes
  robots
  launch
  rviz
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/update_vehicle.py
  scripts/vehicle_position_service.py
  scripts/vehicle_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(xacro REQUIRED)
  find_package(rosunit REQUIRED)
  foreach(T
      test/test_urdf_files.test
      )
     add_rostest(${T})
  endforeach()
endif()