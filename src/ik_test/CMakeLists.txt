
cmake_minimum_required(VERSION 3.22)
project(ik_test LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(CMAKE_BUILD_TYPE "Release")


# ============ 3. include 路径（不要放 .so） ============
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
)


# ============ 4. 查找依赖库 ============
find_package(octomap REQUIRED)
find_package(hpp-fcl REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(pinocchio 3.8.0 REQUIRED)

find_library(CASADI_LIB_FILE 
    NAMES casadi 
    PATHS /usr/local/lib # 尝试根据 config 文件位置推导 lib 路径，或者直接写死如 /usr/local/lib
)


# message(STATUS "PINOCCHIO include: ${PINOCCHIO_INCLUDE_DIRS}")
# message(STATUS "PINOCCHIO libraries: ${pinocchio_LIBRARIES}")
# ============ 5. 源文件 ============
aux_source_directory(${PROJECT_SOURCE_DIR}/src SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})


# ============ 6. 链接库
target_link_libraries(${PROJECT_NAME} PRIVATE
    pinocchio::pinocchio
    ${OCTOMAP_LIBRARIES}
    hpp-fcl::hpp-fcl
    Eigen3::Eigen
    ${CASADI_LIB_FILE}
    qpOASES
)


# ============ 7. include 路径（不要放 .so） ============
target_include_directories(${PROJECT_NAME} PRIVATE
    ${pinocchio_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${HPP_FCL_INCLUDE_DIRS}
    /usr/local/include
)