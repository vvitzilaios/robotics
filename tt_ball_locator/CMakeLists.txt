cmake_minimum_required(VERSION 3.8)
project(tt_ball_locator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(${PROJECT_NAME})
ament_package()


install(DIRECTORY scripts/
      DESTINATION lib/${PROJECT_NAME}
      FILES_MATCHING
      PATTERN "*.py"
      PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
