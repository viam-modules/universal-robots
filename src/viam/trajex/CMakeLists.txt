cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(trajex VERSION 0.1.0
  DESCRIPTION "Viam trajectory execution library"
  HOMEPAGE_URL https://github.com/viamrobotics/trajex
  LANGUAGES CXX
)

# Set minimum macOS deployment target to 14.0 (Sonoma) for mature
# C++20 support in Xcode 15's libc++.
if(APPLE AND NOT CMAKE_OSX_DEPLOYMENT_TARGET)
  set(CMAKE_OSX_DEPLOYMENT_TARGET "14.0")
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

# Guard find_package calls so they're skipped when consumed via
# FetchContent from a parent project that already found these.
if(NOT TARGET Eigen3::Eigen)
  find_package(Eigen3 CONFIG REQUIRED)
endif()
if(NOT TARGET xtl)
  find_package(xtl CONFIG REQUIRED)
endif()
if(NOT TARGET xtensor)
  find_package(xtensor CONFIG REQUIRED)
endif()
if(NOT TARGET Boost::boost)
  find_package(Boost CONFIG REQUIRED COMPONENTS headers)
endif()
if(NOT TARGET jsoncpp_lib)
  find_package(jsoncpp CONFIG REQUIRED)
endif()


#
# Targets
#

# Legacy target: reference implementation from third_party/trajectories submodule
add_library(trajex_totg_legacy)

set_target_properties(trajex_totg_legacy PROPERTIES SYSTEM TRUE)

target_sources(trajex_totg_legacy
  PRIVATE
    src/third_party/trajectories/Trajectory.cpp
    src/third_party/trajectories/Path.cpp
)

target_include_directories(trajex_totg_legacy
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_precompile_headers(trajex_totg_legacy PRIVATE <assert.h>)

target_link_libraries(trajex_totg_legacy
  PUBLIC
    Eigen3::Eigen
)

add_library(viam::trajex::totg-legacy ALIAS trajex_totg_legacy)


# Core target: TOTG implementation
add_library(trajex_totg)

target_sources(trajex_totg
  PRIVATE
    src/viam/trajex/trajex.cpp
    src/viam/trajex/totg/json_serialization.cpp
    src/viam/trajex/totg/observers.cpp
    src/viam/trajex/totg/path.cpp
    src/viam/trajex/totg/trajectory.cpp
    src/viam/trajex/totg/uniform_sampler.cpp
    src/viam/trajex/totg/waypoint_accumulator.cpp
)

target_include_directories(trajex_totg
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(trajex_totg
  PUBLIC
    xtensor
    xtl
  PRIVATE
    Boost::boost
    jsoncpp_lib
)

apply_wall_werror(trajex_totg)

add_library(viam::trajex::totg ALIAS trajex_totg)


#
# Tests
#

enable_testing()

add_executable(trajex-test)

target_sources(trajex-test
  PRIVATE
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
    trajex_totg
    Boost::boost
    trajex_totg_legacy
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

# Install runtime libraries for module packaging
install(
  TARGETS
    trajex_totg_legacy
    trajex_totg
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
