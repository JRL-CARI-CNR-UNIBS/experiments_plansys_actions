cmake_minimum_required(VERSION 3.8)
project(experiments_plansys_actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(plansys2_actions_cost REQUIRED)
find_package(plansys2_action_clients REQUIRED)
find_package(pluginlib REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

include_directories(include
  ${plansys2_action_clients_INCLUDE_DIRS} 
)
set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    nav2_msgs
    nav2_util
    plansys2_domain_expert
    plansys2_executor
    plansys2_planner
    plansys2_problem_expert
    plansys2_pddl_parser
    tf2_ros
    nav2_costmap_2d
    plansys2_actions_cost
    pluginlib
    plansys2_action_clients
    audio_common_msgs
)

add_executable(assist_person_node src/assist_person_node.cpp)
ament_target_dependencies(assist_person_node ${dependencies})

add_executable(attend_person_node src/attend_person_node.cpp)
ament_target_dependencies(attend_person_node ${dependencies})

add_executable(move_action_pure_node src/move_action_pure_node.cpp)
ament_target_dependencies(move_action_pure_node ${dependencies})

add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})

add_executable(patrol_action_node src/patrol_action_node.cpp)
ament_target_dependencies(patrol_action_node ${dependencies})

add_executable(clean_action_node src/clean_action_node.cpp)
ament_target_dependencies(clean_action_node ${dependencies})

add_executable(experiments_controller_node src/experiments_controller_node.cpp)
ament_target_dependencies(experiments_controller_node ${dependencies})

add_executable(experiments_loop_controller_node src/experiments_loop_controller_node.cpp)
ament_target_dependencies(experiments_loop_controller_node ${dependencies})

install(DIRECTORY launch pddl config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  move_action_node
  patrol_action_node
  clean_action_node
  attend_person_node
  assist_person_node
  experiments_controller_node
  experiments_loop_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(experiments_plansys_actions)

install(PROGRAMS
  experiments_plansys_actions/humans_scene_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})

ament_package()
