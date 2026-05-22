cmake_minimum_required(VERSION 3.8)
project(picoxr)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

cmake_host_system_information(RESULT ISA_NAME QUERY OS_PLATFORM)
message(STATUS "OS_PLATFORM: ${ISA_NAME}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(xr_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
set(COAL_DISABLE_HPP_FCL_WARNINGS ON)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

# nlohmann/json
include(FetchContent)
FetchContent_Declare(
  json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(json)

add_executable(talker src/publisher.cpp)
target_compile_features(talker PUBLIC cxx_std_17)

target_compile_definitions(talker PRIVATE COAL_DISABLE_HPP_FCL_WARNINGS)

ament_target_dependencies(talker
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  xr_msgs
  geometry_msgs
  tf2
)

# pico_zme 节点
add_executable(pico_zme src/pico_zme.cpp)
target_compile_features(pico_zme PUBLIC cxx_std_17)
target_compile_definitions(pico_zme PRIVATE COAL_DISABLE_HPP_FCL_WARNINGS)
ament_target_dependencies(pico_zme
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  xr_msgs
  geometry_msgs
  tf2
)

if(NOT UNIX)
  message(FATAL_ERROR "Only support UNIX.") 
endif()

set(ROBOT_SDK "/opt/apps/roboticsservice/SDK")
if(EXISTS ${ROBOT_SDK})
  # message(WARNING "SDK") 
  set(PICORobotSDK_INCLUDE ${ROBOT_SDK}/include)
  set(PICORobotSDK_LIB_PATH ${ROBOT_SDK}/linux/64)
  set(PICORobotSDK_LIB_NAME PXREARobotSDK)
else()
  set(ROBOT_SDK "/opt/apps/picobusinesssuite/SDK/clientso/64")
  if(NOT EXISTS ${ROBOT_SDK})
    message(FATAL_ERROR "robot sdk not found. Please install robot sdk first.") 
  endif()
  set(PICORobotSDK_INCLUDE ${ROBOT_SDK})
  set(PICORobotSDK_LIB_PATH ${ROBOT_SDK})
  set(PICORobotSDK_LIB_NAME PXREARobotSDK)
endif()

foreach(target talker pico_zme)
  target_include_directories(${target} PUBLIC ${PICORobotSDK_INCLUDE})
  target_link_directories(${target} PUBLIC ${PICORobotSDK_LIB_PATH})
endforeach()

if(TARGET pinocchio::pinocchio)
  target_link_libraries(talker pinocchio::pinocchio)
elseif(TARGET pinocchio)
  target_link_libraries(talker pinocchio)
elseif(DEFINED pinocchio_LIBRARIES)
  target_link_libraries(talker ${pinocchio_LIBRARIES})
endif()


if(TARGET Eigen3::Eigen)
  foreach(target talker pico_zme)
    target_link_libraries(${target} Eigen3::Eigen)
  endforeach()
elseif(DEFINED EIGEN3_LIBS)
  foreach(target talker pico_zme)
    target_link_libraries(${target} ${EIGEN3_LIBS})
  endforeach()
endif()


foreach(target talker pico_zme)
  target_link_libraries(${target}
    ${PICORobotSDK_LIB_NAME}
    nlohmann_json::nlohmann_json
  )
endforeach()

install(TARGETS talker pico_zme
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES ${PICORobotSDK_LIB_PATH}/lib${PICORobotSDK_LIB_NAME}.so
        DESTINATION lib)

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

ament_package()