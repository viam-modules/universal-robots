cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(
  universal-robots
  DESCRIPTION "Viam Module to support Universal Robots arms"
  HOMEPAGE_URL https://github.com/viam-modules/universal-robots
  LANGUAGES CXX
)

# Set minimum macOS deployment target to 14.0 (Sonoma). We need macOS 14
# because it ships with Xcode 15's libc++, which has the mature C++20
# support this project requires (concepts, ranges, spaceship operators).
# Targeting 14.0 covers the current macOS release and the two previous
# ones, which seems like a reasonable support window.
#
# This deployment target must be kept in sync with bin/setup.sh and
# bin/build.sh, which set the same version via Conan's os.version
# setting. If you change the version here, update all three locations.
#
# Users can override via: cmake -DCMAKE_OSX_DEPLOYMENT_TARGET=X.Y
#
if(APPLE AND NOT CMAKE_OSX_DEPLOYMENT_TARGET)
  set(CMAKE_OSX_DEPLOYMENT_TARGET "14.0")
endif()

# - `VIAM_UR_USE_WALL_WERROR`
#
# This causes the internal code to compile with `-Wall` and
# `-Werror` flags.
#
option(VIAM_UR_USE_WALL_WERROR "Build with -Wall and -Werror flags" ON)

# Function to apply strict warning flags to a target
function(apply_wall_werror target_name)
  if (NOT VIAM_UR_USE_WALL_WERROR)
    return()
  endif()

  set(GNULIKE_COMPILERS "Clang" "AppleClang" "GNU")
  if (CMAKE_CXX_COMPILER_ID IN_LIST GNULIKE_COMPILERS)
    target_compile_options(${target_name} PRIVATE
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
    target_compile_options(${target_name} PRIVATE /W4 /WX)
  else()
    message(FATAL_ERROR "VIAM_UR_USE_WALL_ERROR is set, but not known how to enable for compiler ID ${CMAKE_CXX_COMPILER_ID}")
  endif()
endfunction()


# If no build type is selected, build optimized but retain debug
# info. This is probably the right build type for packaging and
# release.
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()


# Enforce the C++ standard, and disable extensions
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
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


#
# Obtain dependencies via FetchContent / find_package
#

include(FetchContent)

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
  GIT_TAG releases/v0.25.0 # Keep in sync with Dockerfile
  GIT_SHALLOW TRUE
  SYSTEM
  # Without this patch command, using this as a SYSTEM package results
  # in failed compilation due to the statically included protobuf gens
  # getting in the wrong order. Just delete them, we don't need them.
  PATCH_COMMAND find ./src/viam/api -name "*.pb.h" -type f -exec rm {} +
  FIND_PACKAGE_ARGS
)
FetchContent_MakeAvailable(viam-cpp-sdk)

# xtl and xtensor are provided by viam-cpp-sdk, just need to find them
find_package(xtl CONFIG REQUIRED)
find_package(xtensor CONFIG REQUIRED)

find_package(jsoncpp CONFIG REQUIRED)

find_package(Eigen3 CONFIG REQUIRED)
find_package(viam-cpp-sdk 0.23 CONFIG REQUIRED viamsdk)

# Everything needs threads, and prefer -pthread if available
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Get boost pulled in
find_package(Boost CONFIG REQUIRED
  COMPONENTS
    headers
)

FetchContent_Declare(
  trajex
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/viam/trajex
)
FetchContent_MakeAvailable(trajex)

enable_testing()

#
# Declare targets
#

add_library(viam-ur)

# Compute the relative path from the bindir to the data
# directory. This is relocatable such that if we later install to a
# different prefix it will still be correct, but we need to use the
# full paths to compute it.
file(RELATIVE_PATH VIAM_UR_RELPATH_BINDIR_TO_DATADIR ${CMAKE_INSTALL_FULL_BINDIR} ${CMAKE_INSTALL_FULL_DATADIR})

# Compute the relative path from the bindir to the URCL resources directory.
# URCL installs resources to share/ur_client_library/resources (hardcoded, not
# using CMAKE_INSTALL_DATADIR). This is relocatable even if URCL is installed
# to a different prefix - the relative path will traverse via the common ancestor.

# Determine URCL's install prefix
if(DEFINED universal_robots_client_library_SOURCE_DIR)
  # FetchContent case: both packages install to the same prefix
  set(URCL_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")
else()
  # find_package case: need to determine where URCL is installed
  # Use the location of the imported target's library to work backwards
  get_target_property(URCL_LOCATION ur_client_library::urcl LOCATION)
  # Library is typically in <prefix>/lib, so go up one level
  get_filename_component(URCL_LIB_DIR "${URCL_LOCATION}" DIRECTORY)
  get_filename_component(URCL_INSTALL_PREFIX "${URCL_LIB_DIR}" DIRECTORY)
endif()

set(URCL_RESOURCES_DIR "${URCL_INSTALL_PREFIX}/share/ur_client_library/resources")

file(RELATIVE_PATH VIAM_UR_RELPATH_BINDIR_TO_URCL_RESOURCES
     ${CMAKE_INSTALL_FULL_BINDIR}
     ${URCL_RESOURCES_DIR})

configure_file(
  src/viam/ur/module/ur_arm_config.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/src/viam/ur/module/ur_arm_config.hpp
  @ONLY
)

target_include_directories(viam-ur
  PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/src/viam/ur/module
)

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
    trajex_totg_legacy
    trajex_totg
    urcl
    viam-cpp-sdk::viamapi # Proxy for gRPC
    viam-cpp-sdk::viamsdk
    jsoncpp_lib
)

apply_wall_werror(viam-ur)

install(
  TARGETS
    viam-ur
)


add_executable(universal-robots)

target_sources(universal-robots
  PRIVATE
    src/viam/ur/module/main.cpp
)

target_link_libraries(universal-robots
  PRIVATE
    viam-ur
)

apply_wall_werror(universal-robots)

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

apply_wall_werror(universal-robots-test)

add_test(
  NAME universal-robots
  COMMAND universal-robots-test
)

#
# Installation and packaging
#

configure_file(
    meta.json.in
    ${CMAKE_CURRENT_BINARY_DIR}/meta.json
    @ONLY
)

install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/meta.json
  DESTINATION .
)

install(
  DIRECTORY src/kinematics src/control src/3d_models
  DESTINATION ${CMAKE_INSTALL_DATADIR}/universal-robots
)

set(CPACK_PACKAGE_NAME "viam-universal-robots")
set(CPACK_PACKAGE_FILE_NAME "module")
set(CPACK_GENERATOR "TGZ")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
include(CPack)
