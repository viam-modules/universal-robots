# Toolchain the Viam C++ SDK publishes its darwin binaries with; copied from the
# SDK's etc/conan/macos.profile. bin/build.sh passes it with -pr:a so our
# dependency package IDs match those binaries.
#
# compiler.cppstd is 17 to match the published dependencies; bin/build.sh
# raises only our own package to C++20. Keep in sync when bumping the SDK.
#
# os.version must stay in sync with CMAKE_OSX_DEPLOYMENT_TARGET in CMakeLists.txt.
include(default)

# macOS 14 Sonoma
[settings]
os=Macos
os.version=14.0
arch=armv8
compiler=apple-clang
compiler.version=21
compiler.libcxx=libc++
compiler.cppstd=17
build_type=Release
