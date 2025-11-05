#!/bin/bash
# set -e: exit with errors if anything fails
#     -u: it's an error to use an undefined variable
#     -x: print out every command before it runs
#     -o pipefail: if something in the middle of a pipeline fails, the whole thing fails
set -euxo pipefail

# Get the directory containing this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Assumes bin/build.sh, but otherwise we need git to
# do this, and you might want to build out of a source tarball.
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"


# Otherwise, the C++ SDK build ends up creating two copies of proto and then mixes up which one to use.
cat > protobuf-override.profile << 'EOF'
include(default)
[replace_tool_requires]
protobuf/*: protobuf/<host_version>
EOF

# Dig out the declared version of the module so we can use it for arguments to --build and --requires below.
VIAM_UNIVERSAL_ROBOTS_VERSION=$(conan inspect -vquiet ${REPO_ROOT} --format=json | jq -r '.version')

# Build the viam-universal-robots module
#
# We want a static binary, so we turn off shared. Elect for C++17
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
      -s:a build_type=Release \
      -s:a "viam-cpp-sdk/*:build_type=RelWithDebInfo" \
      -s:a compiler.cppstd=17 \
      -o:a "*:shared=False" \
      -o:a "&:shared=False"

conan create ${REPO_ROOT} \
      --profile=protobuf-override.profile \
      --build=viam-universal-robots/$VIAM_UNIVERSAL_ROBOTS_VERSION \
      -s:a build_type=Release \
      -s:a "viam-cpp-sdk/*:build_type=RelWithDebInfo" \
      -s:a "&:build_type=RelWithDebInfo" \
      -s:a compiler.cppstd=17 \
      -o:a "*:shared=False" \
      -o:a "&:shared=False"

conan install \
      --output-folder=. \
      --profile=protobuf-override.profile \
      --build=never \
      --requires=viam-universal-robots/$VIAM_UNIVERSAL_ROBOTS_VERSION \
      --deployer-folder=. --deployer-package "&" \
      -s:a "viam-cpp-sdk/*:build_type=RelWithDebInfo" \
      -s:a "&:build_type=RelWithDebInfo" \
      -s:a compiler.cppstd=17 \
      -o:a "*:shared=False" \
      -o:a "&:shared=False"
