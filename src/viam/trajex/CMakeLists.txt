cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(trajex VERSION 0.1.0
  DESCRIPTION "Viam trajectory execution library"
  HOMEPAGE_URL https://github.com/viamrobotics/trajex
  LANGUAGES CXX
)

# Detect if we're building trajex as a standalone module
# vs being included via FetchContent by universal-robots
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(TRAJEX_STANDALONE_BUILD ON)
  message(STATUS "Trajex: Standalone build mode")
else()
  set(TRAJEX_STANDALONE_BUILD OFF)
  message(STATUS "Trajex: FetchContent mode (included by parent project)")
endif()

# Set minimum macOS deployment target to 14.0 (Sonoma) for mature
# C++20 support in Xcode 15's libc++.
if(APPLE AND NOT CMAKE_OSX_DEPLOYMENT_TARGET)
  set(CMAKE_OSX_DEPLOYMENT_TARGET "14.0")
endif()

# Configure RPATH for relocatable installs (standalone builds only)
if(TRAJEX_STANDALONE_BUILD)
  set(CMAKE_BUILD_WITH_INSTALL_RPATH false)
  set(CMAKE_BUILD_RPATH_USE_ORIGIN true)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH true)
  set(CMAKE_SKIP_BUILD_RPATH false)

  list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES
       "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}" isSystemDir)
  if("${isSystemDir}" STREQUAL "-1")
    file(RELATIVE_PATH BINTOPREFIXRELPATH
         ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}
         ${CMAKE_INSTALL_PREFIX})
    file(RELATIVE_PATH PREFIXTOLIBRELPATH
         ${CMAKE_INSTALL_PREFIX}
         ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

    if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
      list(PREPEND CMAKE_INSTALL_RPATH
           "@loader_path/${BINTOPREFIXRELPATH}/${PREFIXTOLIBRELPATH}")
    else()
      list(PREPEND CMAKE_INSTALL_RPATH
           "$ORIGIN/${BINTOPREFIXRELPATH}/${PREFIXTOLIBRELPATH}")
    endif()
  endif()
endif()

option(VIAM_TRAJEX_USE_WALL_WERROR "Build with -Wall and -Werror flags" ON)

function(apply_wall_werror target_name)
  if (NOT VIAM_TRAJEX_USE_WALL_WERROR)
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
    message(FATAL_ERROR "VIAM_TRAJEX_USE_WALL_WERROR is set, but not known how to enable for compiler ID ${CMAKE_CXX_COMPILER_ID}")
  endif()
endfunction()

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED True)
if (MSVC)
  string(APPEND CMAKE_CXX_FLAGS " /Zc:__cplusplus")
endif()
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


#
# Dependencies
#

# In standalone mode, use FetchContent to obtain viam-cpp-sdk, which will
# transitively provide xtl and xtensor. In FetchContent mode (included by UR),
# the parent project provides the SDK.
if(TRAJEX_STANDALONE_BUILD)
  include(FetchContent)

  FetchContent_Declare(
    viam-cpp-sdk
    GIT_REPOSITORY https://github.com/viamrobotics/viam-cpp-sdk
    GIT_TAG        releases/v0.25.0
    GIT_SHALLOW    TRUE
    SYSTEM
    # Delete conflicting protobuf headers
    PATCH_COMMAND find ./src/viam/api -name "*.pb.h" -type f -exec rm {} +
    FIND_PACKAGE_ARGS
  )

  FetchContent_MakeAvailable(viam-cpp-sdk)
endif()

find_package(Eigen3 CONFIG REQUIRED)
find_package(xtl CONFIG REQUIRED)
find_package(xtensor CONFIG REQUIRED)

find_package(Boost CONFIG REQUIRED COMPONENTS headers)

# jsoncppConfig.cmake doesn't guard against re-import, so a second
# find_package creates a duplicate imported target and fails. Guard it
# for the FetchContent case where the parent already found jsoncpp.
if(NOT TARGET jsoncpp_lib)
  find_package(jsoncpp CONFIG REQUIRED)
endif()


#
# Targets
#

# Legacy target: reference implementation from third_party/trajectories submodule
add_library(viam-trajex-totg-legacy)

set_target_properties(viam-trajex-totg-legacy PROPERTIES SYSTEM TRUE)

target_sources(viam-trajex-totg-legacy
  PRIVATE
    src/third_party/trajectories/Trajectory.cpp
    src/third_party/trajectories/Path.cpp
)

target_include_directories(viam-trajex-totg-legacy
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_precompile_headers(viam-trajex-totg-legacy PRIVATE <assert.h>)

target_link_libraries(viam-trajex-totg-legacy
  PUBLIC
    Eigen3::Eigen
)

add_library(viam::trajex::totg-legacy ALIAS viam-trajex-totg-legacy)


# Core target: TOTG implementation
add_library(viam-trajex-totg)

target_sources(viam-trajex-totg
  PRIVATE
    src/viam/trajex/trajex.cpp
    src/viam/trajex/totg/json_serialization.cpp
    src/viam/trajex/totg/observers.cpp
    src/viam/trajex/totg/path.cpp
    src/viam/trajex/totg/trajectory.cpp
    src/viam/trajex/totg/uniform_sampler.cpp
    src/viam/trajex/totg/waypoint_accumulator.cpp
    src/viam/trajex/totg/waypoint_utils.cpp
)

target_include_directories(viam-trajex-totg
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(viam-trajex-totg
  PUBLIC
    xtensor
    xtl
  PRIVATE
    Boost::boost
    jsoncpp_lib
)

apply_wall_werror(viam-trajex-totg)

add_library(viam::trajex::totg ALIAS viam-trajex-totg)


# Service layer: Eigen-dependent helpers (always built) and the MLModelService
# wrapper (conditionally built when VIAM_TRAJEX_BUILD_SERVICE is ON).
add_library(viam-trajex-service)

target_sources(viam-trajex-service
  PRIVATE
    src/viam/trajex/service/colinearization.cpp
)

target_include_directories(viam-trajex-service
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(viam-trajex-service
  PUBLIC
    Eigen3::Eigen
    viam::trajex::totg
    viam::trajex::totg-legacy
)

apply_wall_werror(viam-trajex-service)

add_library(viam::trajex::service ALIAS viam-trajex-service)


#
# Service executable and SDK-dependent sources
# Always built (provides testing in both standalone and FetchContent modes)
# Only installed/packaged in standalone mode
#

if(NOT TARGET viam-cpp-sdk::viamsdk)
  find_package(viam-cpp-sdk CONFIG REQUIRED viamsdk)
endif()

target_sources(viam-trajex-service
  PRIVATE
    src/viam/trajex/service/trajex_mlmodel_service.cpp
)

target_link_libraries(viam-trajex-service
  PUBLIC
    viam-cpp-sdk::viamsdk
)

add_executable(trajex-service
  src/viam/trajex/service/main.cpp
)

set_target_properties(trajex-service PROPERTIES OUTPUT_NAME viam-trajex-service)

target_link_libraries(trajex-service
  PRIVATE
    viam::trajex::service
)

apply_wall_werror(trajex-service)

add_executable(trajex-service-test
  src/viam/trajex/service/test/trajex_mlmodel_service.cpp
)

target_link_libraries(trajex-service-test
  PRIVATE
    viam::trajex::service
    Boost::boost
)

apply_wall_werror(trajex-service-test)

add_test(
  NAME trajex-service
  COMMAND trajex-service-test
)


#
# Tests
#

enable_testing()

add_executable(trajex-test)

target_sources(trajex-test
  PRIVATE
    src/viam/trajex/service/test/trajectory_planner.cpp
    src/viam/trajex/totg/test/main.cpp
    src/viam/trajex/totg/test/test_utils.cpp
    src/viam/trajex/totg/test/waypoint_accumulator.cpp
    src/viam/trajex/totg/test/path_basic.cpp
    src/viam/trajex/totg/test/path_coalescing.cpp
    src/viam/trajex/totg/test/path_blends.cpp
    src/viam/trajex/totg/test/path_segments.cpp
    src/viam/trajex/totg/test/path_cursor.cpp
    src/viam/trajex/totg/test/trajectory.cpp
    src/viam/trajex/totg/test/uniform_sampler.cpp
    src/viam/trajex/totg/test/integration.cpp
    src/viam/trajex/totg/test/observers.cpp
)

target_link_libraries(trajex-test
  PRIVATE
    viam::trajex::service
    Boost::boost
    jsoncpp_lib
)

apply_wall_werror(trajex-test)

add_test(
  NAME trajex
  COMMAND trajex-test
)


add_executable(trajex-types-test)

target_sources(trajex-types-test
  PRIVATE
    src/viam/trajex/types/test.cpp
)

target_include_directories(trajex-types-test
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

target_link_libraries(trajex-types-test
  PRIVATE
    Boost::boost
)

apply_wall_werror(trajex-types-test)

add_test(
  NAME trajex-types
  COMMAND trajex-types-test
)


#
# Installation
#

include(GNUInstallDirs)

# Always install libraries (UR uses service library for colinearization)
install(
  TARGETS
  viam-trajex-totg-legacy
  viam-trajex-totg
  viam-trajex-service
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

if(TRAJEX_STANDALONE_BUILD)
  # Install service executable (standalone only)
  install(
    TARGETS
    trajex-service
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )

  # Configure meta.json from template
  configure_file(
    meta.json.in
    ${CMAKE_CURRENT_BINARY_DIR}/meta.json
    @ONLY
  )

  # Install meta.json at module root
  install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/meta.json
    DESTINATION .
  )

  # Set up CPack for trajex module packaging
  set(CPACK_PACKAGE_NAME "viam-trajex-service")
  set(CPACK_PACKAGE_FILE_NAME "module")
  set(CPACK_GENERATOR "TGZ")
  set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
  include(CPack)
endif()
