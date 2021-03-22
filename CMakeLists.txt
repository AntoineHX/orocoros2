cmake_minimum_required(VERSION 3.5)
project(orocoros2)

# find_package(Boost REQUIRED COMPONENTS
#   program_options
# )

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rtt_ros2 REQUIRED)
find_package(rtt_ros2_params REQUIRED)
# find_package(rtt_ros2_std_msgs REQUIRED)

orocos_use_package(ocl-deployment REQUIRED)
orocos_use_package(ocl-taskbrowser REQUIRED)

include_directories(include)
# include_directories(SYSTEM
#   ${Boost_INCLUDE_DIRS}
# )

# simple_ros
# orocos_component(simple_ros
#   src/simple_ros.cpp
# )
# ament_target_dependencies(simple_ros
#   rclcpp
#   rtt_ros2_params
# )
# target_link_libraries(simple_ros
#   rtt_ros2_oe_msgs::rtt_oe_msgs_typekit
#   rtt_ros2_std_msgs::rtt_std_msgs_typekit
# )
# orocos_executable(simple_ros_rtt
#   src/simple_ros_rtt.cpp
#   INSTALL lib/${PROJECT_NAME}
# )
# target_compile_options(simple_ros_rtt PRIVATE "-Wno-unused-parameter")  # RTT library has a bunch of unused parameter warnings
# ament_target_dependencies(simple_ros_rtt
#   rclcpp
# )
# target_link_libraries(simple_ros_rtt Boost::program_options)

# install(DIRECTORY include/
#   DESTINATION include
# )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

rtt_ros2_export_plugin_depend(rtt_ros2_params)
rtt_ros2_export_plugin_depend(rtt_ros2_oe_msgs)
# rtt_ros2_export_plugin_depend(rtt_ros2_std_msgs)

find_package(ament_cmake REQUIRED)

ament_package()

orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS_TARGETS
    rtt_ros2
    rtt_ros2_params
    # rtt_ros2_std_msgs
)
