cmake_minimum_required(VERSION 3.5)
project(software_training_assignment)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#ensure the include directories are included
include_directories(include)

#for the custom service, message and action
rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/Software.msg"
	"srv/Software.srv"
	DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
	)
ament_export_dependencies(rosidl_default_runtime)

#create ament index resource that references the libraries in the binary dir
set(node_plugins "")

#add component that clears any existing turtles as a plugin
add_library(clear_turtles_client SHARED
src/clear_turtles_client.cpp)
target_compile_definitions(clear_turtles_client
PRIVATE "SOFTWARE_TRAINING_ASSIGNMENT_BUILDING_DLL")
ament_target_dependencies(clear_turtles_client
"rclcpp"
"rclcpp_components"
"turtlesim")
rclcpp_components_register_nodes(clear_turtles_client "software_training_assignment::Clear_turtles_client")
set(node_plugins "${node_plugins}software_training_assignment::Clear_turtles_client;$<TARGET_FILE:clear_turtles_client>\n")

#add component that moves turtle1 in a circular motion as a plugin
add_library(circular_move_publisher SHARED
src/circular_move_publisher.cpp)
target_compile_definitions(circular_move_publisher
PRIVATE "SOFTWARE_TRAINING_ASSIGNMENT_BUILDING_DLL")
ament_target_dependencies(circular_move_publisher
"rclcpp"
"rclcpp_components"
"turtlesim"
"std_msgs"
"geometry_msgs")
rclcpp_components_register_nodes(circular_move_publisher "software_training_assignment::Circular_move_publisher")

set(node_plugins "${node_plugins}software_training_assignment::Circular_move_publisher;$<TARGET_FILE:circular_move_publisher\n">)

#add component that spawns stationary_turtle and moving_turtle
add_library(spawn_turtle SHARED
src/spawn_turtle.cpp)
target_compile_definitions(spawn_turtle
PRIVATE "SOFTWARE_TRAINING_ASSIGNMENT_BUILDING_DLL")
ament_target_dependencies(spawn_turtle
"rclcpp"
"rclcpp_components"
"turtlesim")
rclcpp_components_register_nodes(spawn_turtle
"software_training_assignment::Spawn_turtle")

set(node_plugins "${node_plugins}software_training_assignment::Spawn_turtle;$<TARGET_FILE:spawn_turtle\n">)

#add component that resets the moving turtle to its starting position
add_library(reset_moving_turtle_service SHARED
src/reset_moving_turtle_service.cpp)
target_compile_definitions(reset_moving_turtle_service
PRIVATE "SOFTWARE_TRAINING_ASSIGNMENT_BUILDING_DLL")
ament_target_dependencies(reset_moving_turtle_service
"rclcpp"
"rclcpp_components"
"turtlesim"
"std_msgs"
)
rosidl_target_interfaces(reset_moving_turtle_service ${PROJECT_NAME} "ros_idl_typesupport_cpp")
rclcpp_components_register_nodes(reset_moving_turtle_service "software_training_assignment::Reset_moving_turtle_service")
set(node_plugins "${node_plugins}software_training_assignment::Reset_moving_turtle_service;$<TARGET_FILE:reset_moving_turtle_service\n">)

#add component that publishes a custom message 
add_library(custom_publisher SHARED
src/custom_publisher.cpp)
target_compile_definitions(custom_publisher
PRIVATE "SOFTWARE_TRAINING_ASSIGNMENT_BUILDING_DLL")
ament_target_dependencies(custom_publisher
"rclcpp"
"rclcpp_components"
"turtlesim"
"std_msgs"
)
rosidl_target_interfaces(custom_publisher ${PROJECT_NAME} "ros_idl_typesupport_cpp")
rclcpp_components_register_nodes(custom_publisher "software_training_assignment::Custom_publisher")
set(node_plugins "${node_plugins}software_training_assignment::Custom_publisher;$<TARGET_FILE:custom_publisher\n">)

#add action that moves 'moving_turtle'
add_library(moving_turtle_action_server SHARED
src/moving_turtle_action_server.cpp)
target_compile_definitions(moving_turtle_action_server
PRIVATE "SOFTWARE_TRAINING_ASSIGNMENT_BUILDING_DLL")
ament_target_dependencies(moving_turtle_action_server
"rclcpp"
"rclcpp_components"
"rclcpp_action"
"turtlesim"
"std_msgs"
"geometry_msgs"
)
rosidl_target_interfaces(moving_turtle_action_server ${PROJECT_NAME} "ros_idl_typesupport_cpp")
rclcpp_components_register_nodes(reset_moving_turtle_service PLUGIN "software_training_assignment::Moving_turtle_action_server")
set(node_plugins "${node_plugins}software_training_assignment::Moving_turtle_action_server;$<TARGET_FILE:moving_turtle_action_server\n">)

#this tells the compiler where to put the binaries
install(TARGETS
   clear_turtles_client
   circular_move_publisher
   spawn_turtle
   reset_moving_turtle_service
   custom_publisher
   moving_turtle_action_server
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin)
	
#install launch file
install(DIRECTORY
	 launch
	 DESTINATION share/${PROJECT_NAME}
	 )
	 
#install configuration file for param server
install(DIRECTORY
	 config
	 DESTINATION share/${PROJECT_NAME}
	 )

ament_package()

