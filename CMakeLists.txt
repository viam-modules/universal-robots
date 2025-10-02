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


# - `VIAM_UR_DISABLE_APPIMAGE`
#
# Configure the build for not producing an AppImage
#
option(VIAM_UR_DISABLE_APPIMAGE "Disable AppImage builds" OFF)


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

# If we aren't doing an app image build, configure runtime paths for relocatable installs
if (VIAM_UR_DISABLE_APPIMAGE)
  set(CMAKE_BUILD_WITH_INSTALL_RPATH false)
  set(CMAKE_BUILD_RPATH_USE_ORIGIN true)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH true)
  set(CMAKE_SKIP_BUILD_RPATH false)

  list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}" isSystemDir)
  if("${isSystemDir}" STREQUAL "-1")
    # TODO: Is there a way to do this with generator expressions?
    file(RELATIVE_PATH BINTOPREFIXRELPATH ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR} ${CMAKE_INSTALL_PREFIX})
    file(RELATIVE_PATH PREFIXTOLIBRELPATH ${CMAKE_INSTALL_PREFIX} ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

    if (CMAKE_SYSTEM_NAME STREQUAL "Darwin")
      list(PREPEND CMAKE_INSTALL_RPATH "@loader_path/${BINTOPREFIXRELPATH}/${PREFIXTOLIBRELPATH}")
    else()
      list(PREPEND CMAKE_INSTALL_RPATH "$ORIGIN/${BINTOPREFIXRELPATH}/${PREFIXTOLIBRELPATH}")
    endif()
  endif()
endif()


#
# Obtain dependencies via FetchContent / find_package
#

include(FetchContent)

# NOTE: If you change the version below, please also update
# the file `src/control/external_control.urscript` to match.
#
# TODO(RSDK-11618): We should just pull that file out of the repository.
FetchContent_Declare(
  Universal_Robots_Client_Library
  GIT_REPOSITORY https://github.com/UniversalRobots/Universal_Robots_Client_Library
  GIT_TAG        2.2.0
  GIT_SHALLOW TRUE
  SYSTEM
)
FetchContent_MakeAvailable(Universal_Robots_Client_Library)

FetchContent_Declare(
  viam-cpp-sdk
  GIT_REPOSITORY https://github.com/viamrobotics/viam-cpp-sdk
  GIT_TAG releases/v0.19.0 # Keep in sync with Dockerfile
  GIT_SHALLOW TRUE
  SYSTEM
  # Without this patch command, using this as a SYSTEM package results
  # in failed compilation due to the statically included protobuf gens
  # getting in the wrong order. Just delete them, we don't need them.
  PATCH_COMMAND find ./src/viam/api -name "*.pb.h" -type f -exec rm {} +
  FIND_PACKAGE_ARGS
)
FetchContent_MakeAvailable(viam-cpp-sdk)

find_package(Eigen3 3.4 CONFIG REQUIRED)
find_package(viam-cpp-sdk 0.13 CONFIG REQUIRED viamsdk)

# Everything needs threads, and prefer -pthread if available
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Get boost pulled in
find_package(Boost CONFIG REQUIRED
  COMPONENTS
    headers
)


#
# Declare targets
#

add_library(trajectories)

set_target_properties(trajectories PROPERTIES SYSTEM TRUE)

target_sources(trajectories
  PRIVATE
    src/third_party/trajectories/Trajectory.cpp
    src/third_party/trajectories/Path.cpp
)

target_include_directories(trajectories
  INTERFACE
    src
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

if (VIAM_UR_DISABLE_APPIMAGE)

  # Compute the relative path from the bindir to the data
  # directory. This is relocatable such that if we later install to a
  # different prefix it will still be correct, but we need to use the
  # full paths to compute it.
  file(RELATIVE_PATH VIAM_UR_RELPATH_BINDIR_TO_DATADIR ${CMAKE_INSTALL_FULL_BINDIR} ${CMAKE_INSTALL_FULL_DATADIR})

  configure_file(
      src/viam/ur/module/ur_arm_config.hpp.in
      ${CMAKE_CURRENT_BINARY_DIR}/src/viam/ur/module/ur_arm_config.hpp
      @ONLY
  )

  target_include_directories(viam-ur
    PRIVATE
      ${CMAKE_CURRENT_BINARY_DIR}/src/viam/ur/module
  )
endif()

target_sources(viam-ur
  PRIVATE
    src/viam/ur/module/ur_arm.cpp
    src/viam/ur/module/ur_arm_state.cpp
    src/viam/ur/module/ur_arm_state_connected.cpp
    src/viam/ur/module/ur_arm_state_controlled.cpp
    src/viam/ur/module/ur_arm_state_disconnected.cpp
    src/viam/ur/module/ur_arm_state_events.cpp
    src/viam/ur/module/ur_arm_state_independent.cpp
    src/viam/ur/module/utils.cpp
)

# Tell boost::dll to use std::filesystem rather than boost::filesystem.
target_compile_definitions(viam-ur
  PRIVATE
    BOOST_DLL_USE_STD_FS
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
    # TODO: Arguably, these should be `PRIVATE`.
    target_compile_options(viam-ur PUBLIC
      -Wall
      -Werror
      -Wextra
      -pedantic
      -Wconversion
      -Wdouble-promotion
      -Wenum-conversion
      -Wnull-dereference
      -Wunused
      # TODO(RSDK-11300): rm after upgrading boost https://github.com/boostorg/mpl/issues/69
      -Wno-enum-constexpr-conversion
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
    src/viam/ur/module/main.cpp
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
    src/viam/ur/module/test.cpp
)

target_link_libraries(universal-robots-test
  PRIVATE
    viam-ur
)



#
# Installation and packaging
#

if (VIAM_UR_DISABLE_APPIMAGE)
  configure_file(
      meta.json-no-app-image.in
      ${CMAKE_CURRENT_BINARY_DIR}/meta.json
      @ONLY
  )

  install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/meta.json
    DESTINATION .
  )

  install(
    DIRECTORY src/kinematics src/control
    DESTINATION ${CMAKE_INSTALL_DATADIR}/universal-robots
  )

  set(CPACK_PACKAGE_NAME "viam-universal-robots")
  set(CPACK_PACKAGE_FILE_NAME "module")
  set(CPACK_GENERATOR "TGZ")
  set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
  include(CPack)

endif()
