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

# The checked-in profiles describe the toolchain the C++ SDK publishes its
# binaries to viamconan with (C++17, Release, static). Using them keeps our
# dependency package IDs identical to the published ones, so if the viamconan
# remote is configured, dependencies are downloaded rather than built. Only our
# own package (`&`) and the SDK's build type deviate, via the settings below.
if [[ "$(uname -s)" == "Darwin" ]]; then
    PROFILE=${REPO_ROOT}/etc/conan/macos.profile
else
    PROFILE=${REPO_ROOT}/etc/conan/linux.profile
fi

# conan.lock pins the exact dependency versions the SDK release was built
# with. Binaries on viamconan only match those exact versions.
LOCKFILE=${REPO_ROOT}/conan.lock

# Dig out the declared version of the module so we can use it for arguments to --build and --requires below.
# Uses python3 rather than jq because python3 is always present wherever conan
# (a python tool) is installed, while jq is not, e.g. in the C++ SDK's conan images.
VIAM_UNIVERSAL_ROBOTS_VERSION=$(conan inspect -vquiet ${REPO_ROOT} --format=json | python3 -c 'import json, sys; print(json.load(sys.stdin)["version"])')

# Build the viam-universal-robots module
#
# We want a static binary, so we turn off shared. Elect for C++20
# compilation for our package only; dependencies stay on the profile's C++17
# so their package IDs match the published binaries.

# We could just call `build` here with `--build=missing`, but we split this into two steps
# so we can ensure that all our dependencies are built consistently in `Release`, but that
# the actual module build gets built with an override to `RelWithDebInfo`, which we
# don't want to have accidentally affect our dependencies (it makes the build far too large).
# The override itself is derived from https://github.com/conan-io/conan/issues/12656.

conan install ${REPO_ROOT} --update \
      --output-folder=. \
      --profile:all=${PROFILE} \
      --lockfile=${LOCKFILE} \
      --build=missing \
      -s:a "viam-cpp-sdk/*:build_type=RelWithDebInfo" \
      -s:a "&:compiler.cppstd=20" \
      -o:a "*:shared=False" \
      -o:a "&:shared=False" \
      -o:a "boost/*:without_cobalt=True"

conan create ${REPO_ROOT} \
      --profile:all=${PROFILE} \
      --lockfile=${LOCKFILE} \
      --build=viam-universal-robots/$VIAM_UNIVERSAL_ROBOTS_VERSION \
      -s:a "viam-cpp-sdk/*:build_type=RelWithDebInfo" \
      -s:a "&:build_type=RelWithDebInfo" \
      -s:a "&:compiler.cppstd=20" \
      -o:a "*:shared=False" \
      -o:a "&:shared=False" \
      -o:a "boost/*:without_cobalt=True"

conan install \
      --output-folder=. \
      --profile:all=${PROFILE} \
      --lockfile=${LOCKFILE} \
      --lockfile-partial \
      --build=never \
      --requires=viam-universal-robots/$VIAM_UNIVERSAL_ROBOTS_VERSION \
      --deployer-folder=. --deployer-package "&" \
      -s:a "viam-cpp-sdk/*:build_type=RelWithDebInfo" \
      -s:a "&:build_type=RelWithDebInfo" \
      -s:a "&:compiler.cppstd=20" \
      -o:a "*:shared=False" \
      -o:a "&:shared=False" \
      -o:a "boost/*:without_cobalt=True"
