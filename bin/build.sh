#!/bin/bash
# set -e: exit with errors if anything fails
#     -u: it's an error to use an undefined variable
#     -x: print out every command before it runs
#     -o pipefail: if something in the middle of a pipeline fails, the whole thing fails
set -euxo pipefail


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

cat > protobuf-override.profile << 'EOF'
include(default)
[replace_tool_requires]
protobuf/*: protobuf/<host_version>
EOF

VIAM_UNIVERSAL_ROBOTS_VERSION=$(conan inspect -vquiet . --format=json | jq -r '.version')

conan install . --update \
      --profile=protobuf-override.profile \
      --build={missing,outdated} \
      -s:a build_type=Release \
      -s:a "viam-cpp-sdk/*:build_type=RelWithDebInfo" \
      -s:a compiler.cppstd=17 \
      -o:a "*:shared=False" \
      -o:a "&:shared=False"

conan create . \
      --profile=protobuf-override.profile \
      --build=viam-universal-robots/$VIAM_UNIVERSAL_ROBOTS_VERSION \
      -s:a build_type=Release \
      -s:a "viam-cpp-sdk/*:build_type=RelWithDebInfo" \
      -s:a "&:build_type=RelWithDebInfo" \
      -s:a compiler.cppstd=17 \
      -o:a "*:shared=False" \
      -o:a "&:shared=False"

conan install \
      --profile=protobuf-override.profile \
      --requires=viam-universal-robots/$VIAM_UNIVERSAL_ROBOTS_VERSION \
      --deployer-folder=packageme --deployer-package "&" \
      -s:a "viam-cpp-sdk/*:build_type=RelWithDebInfo" \
      -s:a "&:build_type=RelWithDebInfo" \
      -s:a compiler.cppstd=17 \
      -o:a "*:shared=False" \
      -o:a "&:shared=False"
