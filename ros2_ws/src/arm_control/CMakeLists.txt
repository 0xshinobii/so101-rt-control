cmake_minimum_required(VERSION 3.16)
project(arm_control CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Mujoco REQUIRED)
find_package(pinocchio REQUIRED)

# --- Control core: backend + controller + loop (no ROS, no I/O) ---------------
add_library(arm_core
  src/mujoco_backend.cpp
  src/pinocchio_dynamics.cpp
  src/pd_controller.cpp
  src/computed_torque_controller.cpp
  src/payload_mass_rls.cpp
  src/adaptive_computed_torque_controller.cpp
  src/control_loop.cpp
)
target_include_directories(arm_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(arm_core PUBLIC
  mujoco::mujoco
  Eigen3::Eigen
  pinocchio::pinocchio
)

# Gate controller integration on equivalence of the MuJoCo and Pinocchio
# rigid-body terms (gravity, mass matrix, and nonlinear bias).
add_executable(validate_dynamics src/validate_dynamics.cpp)
target_link_libraries(validate_dynamics arm_core)
install(TARGETS validate_dynamics DESTINATION lib/${PROJECT_NAME})

add_executable(validate_payload_estimator src/validate_payload_estimator.cpp)
target_link_libraries(validate_payload_estimator arm_core)
install(TARGETS validate_payload_estimator DESTINATION lib/${PROJECT_NAME})

# --- Steps 1-3: headless runner (bare executable, no ROS dependency) ----------
add_executable(main_headless src/main_headless.cpp)
target_link_libraries(main_headless arm_core)
install(TARGETS main_headless DESTINATION lib/${PROJECT_NAME})

# Phase 3: wall-clock wakeup jitter. Header-only RT setup; no MuJoCo.
add_executable(rt_jitter_bench src/rt_jitter_bench.cpp)
target_include_directories(rt_jitter_bench PRIVATE include)
target_link_libraries(rt_jitter_bench PRIVATE pthread)
install(TARGETS rt_jitter_bench DESTINATION lib/${PROJECT_NAME})

# Phase 7: serial-bus RTT (no MuJoCo).
add_executable(bus_timing src/bus_timing.cpp src/feetech_bus.cpp)
target_include_directories(bus_timing PRIVATE include)
install(TARGETS bus_timing DESTINATION lib/${PROJECT_NAME})

# --- Step 4: ROS 2 node (added when its deps are available) -------------------
find_package(rclcpp QUIET)
find_package(sensor_msgs QUIET)
find_package(arm_msgs QUIET)
if(rclcpp_FOUND AND sensor_msgs_FOUND AND arm_msgs_FOUND)
  add_executable(arm_control_node src/arm_control_node.cpp)
  target_link_libraries(arm_control_node arm_core)
  ament_target_dependencies(arm_control_node rclcpp sensor_msgs arm_msgs)
  target_link_libraries(arm_control_node pthread)
  install(TARGETS arm_control_node DESTINATION lib/${PROJECT_NAME})
  message(STATUS "arm_control: building ROS 2 node arm_control_node")
else()
  message(STATUS "arm_control: ROS 2 deps not found; building headless core only")
endif()

install(DIRECTORY include/ DESTINATION include)

ament_package()
