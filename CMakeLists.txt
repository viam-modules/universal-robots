cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(
  universal-robots
  DESCRIPTION "Viam Module to support Universal Robots arms"
  HOMEPAGE_URL https://github.com/viam-modules/universal-robots
  LANGUAGES CXX
)

# - `VIAM_UR_USE_WALL_WERROR`
#
# This causes the internal code to compile with `-Wall` and
# `-Werror` flags.
#
option(VIAM_UR_USE_WALL_WERROR "Build with -Wall and -Werror flags" ON)


# If no build type is selected, build optimized but retain debug
# info. This is probably the right build type for packaging and
# release.
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()


# Enforce the C++ standard, and disable extensions
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED True)
if (MSVC)
  # https://discourse.cmake.org/t/set-cmake-cxx-standard-should-set-zc-cplusplus-for-msvc/1876
  string(APPEND CMAKE_CXX_FLAGS " /Zc:__cplusplus")
endif()
set(CMAKE_CXX_EXTENSIONS OFF)


# Produce a compilation database when generating for a build
# system that is able to make one.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# Produce GNU-style variables for installation, and default the
# installation directory to be local to the build. If you intend to
# install elsewhere, pass an explicit argument to CMAKE_INSTALL_PREFIX
# on the command line:
#
# cmake ... -DCMAKE_INSTALL_PREFIX=$HOME/opt
#
if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "..." FORCE)
endif()
include(GNUInstallDirs)


# Obtain dependencies via FetchContent / find_package
include(FetchContent)

FetchContent_Declare(
  Universal_Robots_Client_Library
  GIT_REPOSITORY https://github.com/UniversalRobots/Universal_Robots_Client_Library
  GIT_TAG        2.0.0
  GIT_SHALLOW TRUE
  SYSTEM
)
FetchContent_MakeAvailable(Universal_Robots_Client_Library)

FetchContent_Declare(
  viam-cpp-sdk
  GIT_REPOSITORY https://github.com/viamrobotics/viam-cpp-sdk
  GIT_TAG releases/v0.13.2 # Keep in sync with Dockerfile
  GIT_SHALLOW TRUE
  SYSTEM
  FIND_PACKAGE_ARGS
)
FetchContent_MakeAvailable(viam-cpp-sdk)

find_package(Eigen3 3.3 CONFIG REQUIRED)
find_package(viam-cpp-sdk 0.13 CONFIG REQUIRED viamsdk)

# Everything needs threads, and prefer -pthread if available
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)


# Declare targets
add_library(trajectories)
set_target_properties(trajectories PROPERTIES SYSTEM TRUE)
target_sources(trajectories
  PRIVATE
    src/third_party/trajectories/Trajectory.cpp
    src/third_party/trajectories/Path.cpp
)
target_link_libraries(trajectories
  PUBLIC
    Eigen3::Eigen
)
install(
  TARGETS
    trajectories
)

add_library(viam-ur)
target_sources(viam-ur
  PRIVATE
    src/viam/ur/ur_arm.cpp
)
target_link_libraries(viam-ur
  PUBLIC
    trajectories
    urcl
    viam-cpp-sdk::viamsdk
)
install(
  TARGETS
    viam-ur
)

# If we have opted into more extensive warnings as errors, apply them now
if (VIAM_UR_USE_WALL_WERROR)
  set(GNULIKE_COMPILERS "Clang" "AppleClang" "GNU")
  if (CMAKE_CXX_COMPILER_ID IN_LIST GNULIKE_COMPILERS)
    target_compile_options(viam-ur PRIVATE
      -Wall
      -Werror
      -Wextra
      -pedantic
      -Wconversion
      -Wdouble-promotion
      -Wenum-conversion
      -Wnull-dereference
      -Wunused
    )
  elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(viam-ur PRIVATE /W4 /WX)
  else()
    message(FATAL_ERROR "VIAM_UR_USE_WALL_ERROR is set, but not known how to enable for compiler ID ${CMAKE_CXX_COMPILER_ID}")
  endif()
endif()


add_executable(universal-robots)
target_sources(universal-robots
  PRIVATE
    src/viam/ur/main.cpp
)
target_link_libraries(universal-robots
  PRIVATE
    viam-ur
)
install(
  TARGETS
    universal-robots
)

add_executable(universal-robots-test)
target_sources(universal-robots-test
  PRIVATE
    src/viam/ur/test.cpp
)
target_link_libraries(universal-robots-test
  PRIVATE
    viam-ur
)
