cmake_minimum_required(VERSION 3.8)
project(ros2_servo)

set(BUILD_SHARED_LIBS OFF)  # we're not using standard libraries, so turn shared libraries off

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# we're using the idl system to generate our interface, so we need to find that package
find_package(rosidl_default_generators REQUIRED)

# we also need to tell the generator what our messages are
rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/Servo.msg"
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()

# script to create firmware workspace setup
install(
	PROGRAMS
	scripts/create_fwws.sh
	DESTINATION lib/${PROJECT_NAME}
)
