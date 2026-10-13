cmake_minimum_required(VERSION 3.8)
project(volleyball_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(chassis_controllers REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(universal_utils REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(fmt REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/StewartControl.msg"

        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(
        cpp_typesupport_target
        ${PROJECT_NAME}
        "rosidl_typesupport_cpp"
)

add_executable(dm_stewart_controller src/dm_stewart_controller.cpp src/stewart_ik.cpp)
ament_target_dependencies(dm_stewart_controller rclcpp std_msgs sensor_msgs custom_msgs tf2 tf2_ros fmt)
target_link_libraries(
        dm_stewart_controller
        ${cpp_typesupport_target}
        Eigen3::Eigen
        fmt
)
target_include_directories(dm_stewart_controller PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
)
target_compile_features(dm_stewart_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS dm_stewart_controller
        DESTINATION lib/${PROJECT_NAME})

add_executable(volleyball_hub_controller src/volleyball_hub_controller.cpp)
ament_target_dependencies(volleyball_hub_controller rclcpp std_msgs sensor_msgs custom_msgs tf2 tf2_ros fmt chassis_controllers universal_utils)
target_link_libraries(
        volleyball_hub_controller
        ${cpp_typesupport_target}
        Eigen3::Eigen
        fmt
)
target_include_directories(volleyball_hub_controller PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
)
target_compile_features(volleyball_hub_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS volleyball_hub_controller
        DESTINATION lib/${PROJECT_NAME})

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

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
