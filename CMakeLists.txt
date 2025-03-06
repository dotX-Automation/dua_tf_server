cmake_minimum_required(VERSION 3.10)
project(dua_tf_server)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(dua_app_management REQUIRED)
find_package(dua_common_interfaces REQUIRED)
find_package(dua_geometry_interfaces REQUIRED)
find_package(dua_node_cpp REQUIRED)
find_package(dua_qos_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# TF Server library
file(GLOB DUA_TF_SERVER_SOURCES
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/dua_tf_server/*.cpp"
)
add_library(dua_tf_server SHARED ${DUA_TF_SERVER_SOURCES})
target_compile_features(dua_tf_server PUBLIC c_std_99 cxx_std_17)
target_compile_definitions(dua_tf_server PRIVATE "DUA_TF_SERVER_BUILDING_LIBRARY")
target_include_directories(dua_tf_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(dua_tf_server
  "dua_common_interfaces"
  "dua_geometry_interfaces"
  "dua_node_cpp"
  "dua_qos_cpp"
  "geometry_msgs"
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
)
rclcpp_components_register_nodes(dua_tf_server "dua_tf_server::TFServerNode")

# TF Server app
add_executable(dua_tf_server_app
  src/dua_tf_server_app.cpp)
target_include_directories(dua_tf_server_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(dua_tf_server_app dua_tf_server)
ament_target_dependencies(dua_tf_server_app
  "dua_app_management"
  "rclcpp"
)

# Install TF Server library
install(
  TARGETS dua_tf_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install TF Server app
install(TARGETS dua_tf_server_app
  DESTINATION lib/${PROJECT_NAME})

# Install Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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
