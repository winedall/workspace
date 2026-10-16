cmake_minimum_required(VERSION 3.5)
project(yesense_std_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(src/serial/include)

set(SERIAL_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/serial)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(SERIAL_LIB_FILE ${SERIAL_LIB_DIR}/libserial_arm.so)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  set(SERIAL_LIB_FILE ${SERIAL_LIB_DIR}/libserial_x86.so)
else()
  message(FATAL_ERROR "Unsupported architecture for yesense libserial: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

if(NOT EXISTS ${SERIAL_LIB_FILE})
  message(FATAL_ERROR "yesense libserial not found: ${SERIAL_LIB_FILE}")
endif()

message(STATUS "Using yesense libserial: ${SERIAL_LIB_FILE}")

add_library(yesense_serial SHARED IMPORTED GLOBAL)
set_target_properties(yesense_serial PROPERTIES
  IMPORTED_LOCATION ${SERIAL_LIB_FILE}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yesense_interface REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(yesense_node_publisher 
				src/yesense_node.cpp
				src/yesense_std_out_decoder.cpp
				src/yesense_decoder.cpp
				)

ament_target_dependencies(yesense_node_publisher rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs yesense_interface)
#ament_target_dependencies(yesense_node_publisher rclcpp std_msgs)

target_link_libraries(yesense_node_publisher 
						yesense_serial
						)

add_executable(yesense_node_subscriber 
				src/yesense_node_subscriber_example.cpp
				)
				
ament_target_dependencies(yesense_node_subscriber rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs yesense_interface)				

install(DIRECTORY src/serial/
		DESTINATION lib/${PROJECT_NAME}/serial
		)

install(DIRECTORY launch
  	DESTINATION share/${PROJECT_NAME}
  	)

install(DIRECTORY config/
  	DESTINATION share/${PROJECT_NAME}/config
  	)

install(TARGETS
  yesense_node_publisher
  yesense_node_subscriber
  DESTINATION lib/${PROJECT_NAME}
  )

install(FILES ${SERIAL_LIB_FILE}
        DESTINATION lib
        RENAME libserial.so
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
