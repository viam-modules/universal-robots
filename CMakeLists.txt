cmake_minimum_required(VERSION 3.7 FATAL_ERROR)

project(ur5earm LANGUAGES CXX)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

include(FetchContent)
FetchContent_Declare(
  viam-cpp-sdk
  GIT_REPOSITORY https://github.com/viamrobotics/viam-cpp-sdk.git
  GIT_TAG main
  # SOURCE_DIR ${CMAKE_SOURCE_DIR}/../viam-cpp-sdk
  CMAKE_ARGS -DVIAMCPPSDK_USE_DYNAMIC_PROTOS=ON
  FIND_PACKAGE_ARGS
)
FetchContent_MakeAvailable(viam-cpp-sdk)

include_directories(/usr/local/lib/cmake/viam-cpp-sdk)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
# Find the ur_client_library package
# find_package(ur_client_library REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/Universal_Robots_Client_Library/include ${PROJECT_SOURCE_DIR}/trajectories)
link_directories(Universal_Robots_Client_Library/build)

FILE(GLOB sources *.cpp)
add_executable(ur5earm-x86 ${sources} src/UR5e-Arm.cpp src/UR5e-Arm.hpp trajectories/Path.cpp trajectories/Trajectory.cpp)
target_link_libraries(ur5earm-x86 PRIVATE viam-cpp-sdk::viamsdk Eigen3::Eigen urcl)
