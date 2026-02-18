#!/bin/bash
# set -e: exit with errors if anything fails
#     -u: it's an error to use an undefined variable
#     -x: print out every command before it runs
#     -o pipefail: if something in the middle of a pipeline fails, the whole thing fails
set -euxo pipefail

# Get the directory containing this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Trajex root is one level up from bin/
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"


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
# This must stay in sync with CMakeLists.txt and bin/setup.sh. If you
# change the version here, update all three locations.
#
# We query the actual Conan profile to determine whether we're targeting
# macOS (the host platform where binaries will run), then add os.version
# to the host context if so.
#
HOST_OS=$(conan profile show -pr protobuf-override.profile -cx host --format=json 2>/dev/null | jq -r '.host.settings.os // empty')
MACOS_SETTINGS=""
[[ "$HOST_OS" == "Macos" ]] && MACOS_SETTINGS="-s:a os.version=14.0"

# Dig out the declared version of the module so we can use it for arguments to --build and --requires below.
VIAM_TRAJEX_VERSION=$(conan inspect -vquiet ${REPO_ROOT} --format=json | jq -r '.version')

# Build the viam-trajex-service module
#
# We want a static binary, so we turn off shared. Elect for C++20
# compilation, since it seems some of the dependencies we pick mandate
# it anyway.

# We could just call `build` here with `--build=missing`, but we split this into two steps
# so we can ensure that all our dependencies are built consistently in `Release`, but that
# the actual module build gets built with an override to `RelWithDebInfo`, which we
# don't want to have accidentally affect our dependencies (it makes the build far too large).
# The override itself is derived from https://github.com/conan-io/conan/issues/12656.

conan install ${REPO_ROOT} --update \
      --output-folder=. \
      --profile=protobuf-override.profile \
      --build=missing \
      $MACOS_SETTINGS \
      -s:a build_type=Release \
      -s:a "viam-cpp-sdk/*:build_type=RelWithDebInfo" \
      -s:a compiler.cppstd=20 \
      -o:a "*:shared=False" \
      -o:a "&:shared=False"

conan create ${REPO_ROOT} \
      --profile=protobuf-override.profile \
      --build=viam-trajex-service/$VIAM_TRAJEX_VERSION \
      $MACOS_SETTINGS \
      -s:a build_type=Release \
      -s:a "viam-cpp-sdk/*:build_type=RelWithDebInfo" \
      -s:a "&:build_type=RelWithDebInfo" \
      -s:a compiler.cppstd=20 \
      -o:a "*:shared=False" \
      -o:a "&:shared=False"

conan install \
      --output-folder=. \
      --profile=protobuf-override.profile \
      --build=never \
      --requires=viam-trajex-service/$VIAM_TRAJEX_VERSION \
      --deployer-folder=. --deployer-package "&" \
      $MACOS_SETTINGS \
      -s:a "viam-cpp-sdk/*:build_type=RelWithDebInfo" \
      -s:a "&:build_type=RelWithDebInfo" \
      -s:a compiler.cppstd=20 \
      -o:a "*:shared=False" \
      -o:a "&:shared=False"
