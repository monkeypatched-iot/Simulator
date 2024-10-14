cmake_minimum_required(VERSION 3.8)
project(uno)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY sdf/
  DESTINATION share/${PROJECT_NAME}/sdf)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes)

install(DIRECTORY meshes/rack/meshes
  DESTINATION share/${PROJECT_NAME}/meshes/rack/meshes)

install(DIRECTORY meshes/rack/thumbnails
  DESTINATION share/${PROJECT_NAME}/meshes/rack/thumbnails)

install(DIRECTORY meshes/box/meshes
  DESTINATION share/${PROJECT_NAME}/meshes/box/meshes)

install(DIRECTORY meshes/box/thumbnails
  DESTINATION share/${PROJECT_NAME}/meshes/box/thumbnails)

install(DIRECTORY meshes/man/meshes
  DESTINATION share/${PROJECT_NAME}/meshes/man/meshes)

install(DIRECTORY meshes/man/thumbnails
  DESTINATION share/${PROJECT_NAME}/meshes/man/thumbnails)

install(DIRECTORY meshes/man/materials
  DESTINATION share/${PROJECT_NAME}/meshes/man/materiials)

ament_package()
