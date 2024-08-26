cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(
  universal-robots
  DESCRIPTION "Viam Module to support Universal Robots arms"
  HOMEPAGE_URL https://github.com/viam-modules/universal-robots
  LANGUAGES CXX
)
set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(
  viam-cpp-sdk
  GIT_REPOSITORY https://github.com/viamrobotics/viam-cpp-sdk.git
  GIT_TAG main
  CMAKE_ARGS -DVIAMCPPSDK_USE_DYNAMIC_PROTOS=ON
  FIND_PACKAGE_ARGS
)
FetchContent_MakeAvailable(viam-cpp-sdk)

include_directories(/usr/local/lib/cmake/viam-cpp-sdk)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/Universal_Robots_Client_Library/include ${PROJECT_SOURCE_DIR}/trajectories)
link_directories(Universal_Robots_Client_Library/build)

FILE(GLOB sources *.cpp)
add_executable(universal-robots ${sources} src/UR5e-Arm.cpp src/UR5e-Arm.hpp trajectories/Path.cpp trajectories/Trajectory.cpp)
target_link_libraries(universal-robots PRIVATE viam-cpp-sdk::viamsdk Eigen3::Eigen urcl)
