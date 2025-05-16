cmake_minimum_required(VERSION 3.0.2)
project(arx_x7_controller)

find_package(catkin REQUIRED COMPONENTS
        arm_control
        arx5_arm_msg
        roscpp
        roslib
        std_msgs
        sensor_msgs
        message_generation
        robot_state_publisher
        urdf
        kdl_parser
)

add_service_files(
  FILES
  Pose3D.srv
  LeftPose.srv
  RightPose.srv
)

#add_message_files(
#  FILES
#  PosCmd.msg
#  JointInformation.msg
#  JointControl.msg
#  RobotCmd.msg
#  RobotStatus.msg
#)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS arm_control arx5_arm_msg roscpp std_msgs sensor_msgs robot_state_publisher urdf kdl_parser message_runtime rospy
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# 添加共享库路径
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# 添加可执行文件
add_executable(x7_controller src/x7_controller.cpp)
target_include_directories(x7_controller PUBLIC lib/arx_hardware_interface/include lib/arx_x7_src/include)
target_link_libraries(x7_controller ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_x7_src/libarx_x7_src.so)
add_dependencies(x7_controller  ${catkin_EXPORTED_TARGETS})

add_executable(keyboard src/keyboard.cpp)
target_include_directories(keyboard PUBLIC lib/arx_hardware_interface/include lib/arx_x7_src/include)
target_link_libraries(keyboard ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_x7_src/libarx_x7_src.so)
add_dependencies(keyboard  ${catkin_EXPORTED_TARGETS})

add_executable(task1_action src/task1_action.cpp src/ros_control.cpp)
target_include_directories(task1_action PUBLIC include lib/arx_hardware_interface/include lib/arx_x7_src/include)
target_link_libraries(task1_action ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_x7_src/libarx_x7_src.so)
add_dependencies(task1_action  ${catkin_EXPORTED_TARGETS})

add_executable(task2_action src/task2_action.cpp src/ros_control.cpp)
target_include_directories(task2_action PUBLIC include lib/arx_hardware_interface/include lib/arx_x7_src/include)
target_link_libraries(task2_action ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_x7_src/libarx_x7_src.so)
add_dependencies(task2_action  ${catkin_EXPORTED_TARGETS})

add_executable(task3_pizza_action src/task3_pizza_action.cpp src/ros_control.cpp)
target_include_directories(task3_pizza_action PUBLIC include lib/arx_hardware_interface/include lib/arx_x7_src/include)
target_link_libraries(task3_pizza_action ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_x7_src/libarx_x7_src.so)
add_dependencies(task3_pizza_action  ${catkin_EXPORTED_TARGETS})

add_executable(task3_container_action src/task3_container_action.cpp src/ros_control.cpp)
target_include_directories(task3_container_action PUBLIC include lib/arx_hardware_interface/include lib/arx_x7_src/include)
target_link_libraries(task3_container_action ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_x7_src/libarx_x7_src.so)
add_dependencies(task3_container_action  ${catkin_EXPORTED_TARGETS})

add_executable(ros_control src/ros_control.cpp)
target_include_directories(ros_control PUBLIC lib/arx_hardware_interface/include lib/arx_x7_src/include)
target_link_libraries(ros_control ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_x7_src/libarx_x7_src.so)
add_dependencies(ros_control  ${catkin_EXPORTED_TARGETS})

add_executable(arm_state_publisher src/arm_state_publisher.cpp)
target_include_directories(arm_state_publisher PUBLIC lib/arx_hardware_interface/include lib/arx_x7_src/include)
target_link_libraries(arm_state_publisher ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_x7_src/libarx_x7_src.so)
add_dependencies(arm_state_publisher  ${catkin_EXPORTED_TARGETS})


install(TARGETS
        x7_controller
        DESTINATION lib/${PROJECT_NAME})

install(FILES lib/arx_x7_src/libarx_x7_src.so
        DESTINATION lib/
)
