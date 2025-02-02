cmake_minimum_required(VERSION 3.5)
project(kroad_planning_utm_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# Specify the include directories for your package
include_directories(include)

set(depend
  rclcpp
  std_msgs
  geometry_msgs
)

# Build
add_executable(relative_2_UTM src/relative_2_UTM.cpp)
ament_target_dependencies(relative_2_UTM ${depend}) 

add_executable(narrow_2_UTM src/narrow_2_UTM.cpp)
ament_target_dependencies(narrow_2_UTM ${depend}) 



# Install
install(TARGETS
  relative_2_UTM
  narrow_2_UTM
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
