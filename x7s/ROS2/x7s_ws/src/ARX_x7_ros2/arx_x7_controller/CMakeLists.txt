cmake_minimum_required(VERSION 3.8)
project(arx_x7_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(arm_control REQUIRED)
find_package(arx5_arm_msg REQUIRED)

find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

find_library(ARX_SRC_LIB
    NAMES arx_x7_src
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_x7_src
    NO_DEFAULT_PATH  # 禁止在其他路径中搜索
)

# 确保库文件被找到
if (NOT ARX_SRC_LIB)
    message(FATAL_ERROR "library not found!")
endif()

# 添加可执行文件
add_executable(x7_controller src/x7_controller.cpp)
target_include_directories(x7_controller PUBLIC lib/arx_hardware_interface/include lib/arx_x7_src/include)
target_link_libraries(x7_controller ${ARX_SRC_LIB})
ament_target_dependencies(x7_controller rclcpp ament_index_cpp std_msgs arx5_arm_msg arm_control tf2 tf2_msgs)

# 添加可执行文件
add_executable(keyboard src/keyboard.cpp)
ament_target_dependencies(keyboard rclcpp std_msgs arx5_arm_msg)

install(TARGETS
    x7_controller
    keyboard
  DESTINATION lib/${PROJECT_NAME})

install(FILES ${ARX_SRC_LIB}
  DESTINATION lib/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py")

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml")

install(FILES x7sRIGHT.urdf
        DESTINATION share/${PROJECT_NAME}
)

install(FILES X7Sleft1.urdf
        DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()