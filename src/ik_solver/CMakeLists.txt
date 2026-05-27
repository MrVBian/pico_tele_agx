cmake_minimum_required(VERSION 3.22)
project(ik_solver LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_BUILD_TYPE "Release")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
# 运动学与数学库
find_package(pinocchio 3.8.0 REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
# 新增依赖：octomap, hpp-fcl, casadi, qpOASES
find_package(octomap REQUIRED)
find_package(hpp-fcl REQUIRED)

# 查找 casadi 和 qpOASES 库
find_library(CASADI_LIBRARY NAMES casadi PATHS /usr/local/lib /usr/lib)
find_library(QPOASES_LIBRARY NAMES qpOASES PATHS /usr/local/lib /usr/lib)

# executable
add_executable(${PROJECT_NAME} src/main.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# 使用关键字签名链接所有依赖
target_link_libraries(${PROJECT_NAME} PRIVATE
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    pinocchio::pinocchio
    ${OCTOMAP_LIBRARIES}
    hpp-fcl::hpp-fcl
    Eigen3::Eigen
    ${CASADI_LIBRARY}
    ${QPOASES_LIBRARY}
)

# include 路径
target_include_directories(${PROJECT_NAME} PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
    ${pinocchio_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${HPP_FCL_INCLUDE_DIRS}
    /usr/local/include
)

# install
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()