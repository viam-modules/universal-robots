#!/bin/bash
# set -e: exit with errors if anything fails
#     -u: it's an error to use an undefined variable
#     -x: print out every command before it runs
#     -o pipefail: if something in the middle of a pipeline fails, the whole thing fails
#
set -euxo pipefail

# Get the directory containing this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Trajex root is one level up from bin/
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

conan profile detect || echo "Conan profile already exists"

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

# NOTE: If you change this version, also change it in the `conanfile.py` requirements
# and in the Dockerfile and CMakeLists.txt
git checkout releases/v0.25.0

# Export the recipe to the cache so we can skip rebuilds gracefully
conan export .

# Otherwise, the C++ SDK build ends up creating two copies of proto and then mixes up which one to use.
cat > protobuf-override.profile << 'EOF'
include(default)
[replace_tool_requires]
protobuf/*: protobuf/<host_version>
EOF

# Set macOS deployment target to 14.0 (Sonoma) if building for macOS.
# We need macOS 14 because it ships with Xcode 15's libc++, which has
# mature support for the C++20 features this project uses (concepts,
# ranges, spaceship operators).
#
# This must stay in sync with CMakeLists.txt and bin/build.sh. If you
# change the version here, update all three locations.
#
# We query the actual Conan profile to determine whether we're targeting
# macOS (the host platform where binaries will run), then add os.version
# to the host context if so.
#
HOST_OS=$(conan profile show -pr protobuf-override.profile -cx host --format=json 2>/dev/null | jq -r '.host.settings.os // empty')
MACOS_SETTINGS=""
[[ "$HOST_OS" == "Macos" ]] && MACOS_SETTINGS="-s:a os.version=14.0"

# Dig out the declared version of the SDK so we can use it for arguments to --build and --requires below.
VIAM_CPP_SDK_VERSION=$(conan inspect -vquiet . --format=json | jq -r '.version')


# Build the C++ SDK repo
#
# We want a static binary, so we turn off shared. Elect for C++20
# compilation, since it seems some of the dependencies we pick mandate
# it anyway.

conan install --update \
      --profile=protobuf-override.profile \
      --build=missing \
      --requires=viam-cpp-sdk/${VIAM_CPP_SDK_VERSION} \
      $MACOS_SETTINGS \
      -s:a build_type=Release \
      -s:a "&:build_type=RelWithDebInfo" \
      -s:a compiler.cppstd=20 \
      -o:a "*:shared=False" \
      -o:a "&:shared=False"
