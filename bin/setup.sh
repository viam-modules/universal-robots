#!/bin/bash
# set -e: exit with errors if anything fails
#     -u: it's an error to use an undefined variable
#     -x: print out every command before it runs
#     -o pipefail: if something in the middle of a pipeline fails, the whole thing fails
#
set -euxo pipefail

if [ ! -d "tmp_cpp_sdk/viam-cpp-sdk" ]; then
  # Clone the C++ SDK repo
  mkdir -p tmp_cpp_sdk
  pushd tmp_cpp_sdk
  git clone https://github.com/viamrobotics/viam-cpp-sdk.git
  pushd viam-cpp-sdk
else
  pushd tmp_cpp_sdk
  pushd viam-cpp-sdk
fi

# NOTE: If you change this version, also change it in the `conanfile.py` requirements,
# conan.lock, the Dockerfile, and CMakeLists.txt
git checkout releases/v0.44.0

# Export the recipe to the cache. The C++ SDK is not on conancenter, so this is
# what lets bin/build.sh resolve it without the viamconan remote. Exporting from
# the release tag yields the same recipe revision that conan.lock pins.
conan export .

# By default, add the viamconan remote, which hosts prebuilt binaries of the SDK
# and its heavy dependencies, so bin/build.sh downloads them instead of spending
# about an hour compiling them. This changes your global conan configuration.
# Set VIAM_CONAN_NO_REMOTE=1 to skip it and build everything from conancenter
# sources instead.
if [[ "${VIAM_CONAN_NO_REMOTE:-0}" != "1" ]]; then
  # --index 0 searches viamconan ahead of conancenter.
  conan remote add viamconan https://viam.jfrog.io/artifactory/api/conan/viamconan --index 0 --force
fi
