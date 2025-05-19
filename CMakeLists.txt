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
  Universal_Robots_Client_Library
  GIT_REPOSITORY https://github.com/UniversalRobots/Universal_Robots_Client_Library
  GIT_TAG        2.0.0
  EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(Universal_Robots_Client_Library)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(viam-cpp-sdk CONFIG REQUIRED viamsdk)

# add_compile_options(-Wall -Wextra -pedantic -Wno-unused-parameter -Wall -Wextra -Wpedantic -Wconversion  -Wnull-dereference -Wdouble-promotion -Wno-switch-enum -Wshadow -Wunused -Wenum-conversion -Wno-unused-function -Wuninitialized)
add_executable(universal-robots main.cpp src/ur5e_arm.cpp trajectories/Path.cpp trajectories/Trajectory.cpp)
add_executable(universal-robots-test test.cpp src/ur5e_arm.cpp trajectories/Path.cpp trajectories/Trajectory.cpp)
target_link_libraries(universal-robots PRIVATE viam-cpp-sdk::viamsdk Eigen3::Eigen urcl)
target_link_libraries(universal-robots-test PRIVATE viam-cpp-sdk::viamsdk Eigen3::Eigen urcl)
