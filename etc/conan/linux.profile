# Toolchain the Viam C++ SDK publishes its Linux binaries with: the default
# profile of the ghcr.io/viamrobotics/cpp-sdk-conan-ubuntu:jammy-2026-11.00
# image that CI builds in.
# bin/build.sh passes it with -pr:a so our dependency package IDs match those
# binaries. A `conan profile detect` profile often differs (e.g. cppstd=gnu17),
# which silently misses the cache and rebuilds everything from source.
#
# compiler.cppstd is 17 to match the published dependencies; bin/build.sh
# raises only our own package to C++20. gcc 11 is Ubuntu Jammy's system
# compiler; building on Jammy keeps the module's glibc requirement low enough
# to run on Jammy, Bookworm, and newer. Keep in sync when bumping the SDK.
include(default)

[settings]
os=Linux
compiler=gcc
compiler.version=11
compiler.libcxx=libstdc++11
compiler.cppstd=17
build_type=Release
