#!/bin/bash
# set -e: exit with errors if anything fails
#     -u: it's an error to use an undefined variable
#     -x: print out every command before it runs
#     -o pipefail: if something in the middle of a pipeline fails, the whole thing fails
set -euxo pipefail


# Build the viam-orbbec module
#
# We want a static binary, so we turn off shared. Elect for C++17
# compilation, since it seems some of the dependencies we pick mandate
# it anyway.

# We could just call `build` here with `--build=missing`, but we split this into two steps
# so we can ensure that all our dependencies are built consistently in `Release`, but that
# the actual module build gets built with an override to `RelWithDebInfo`, which we
# don't want to have accidentally affect our dependencies (it makes the build far too large).
# The override itself is derived from https://github.com/conan-io/conan/issues/12656.
#
if [ -f "./venv/bin/activate" ]; then
  echo 'sourcing virtual env'
  source ./venv/bin/activate
fi


cat > protobuf-override.profile << 'EOF'
include(default)

[replace_requires]
protobuf/*: protobuf/5.27.0

[replace_tool_requires]
protobuf/*: protobuf/5.27.0
EOF

#       -o:a "viam-cpp-sdk/*:shared=False" \
#       -s:a "&:build_type=RelWithDebInfo"
conan install . --update \
      --profile=protobuf-override.profile \
      --build={missing,outdated} \
      -s:a build_type=Release \
      -s:a compiler.cppstd=17 \


#      -o:a "viam-cpp-sdk/*:shared=False" \
#      -s:a "viam-cpp-sdk/*:build_type=RelWithDebInfo" \
conan build . \
      --profile=protobuf-override.profile \
      --build=none \
       -s:a build_type=Release \
      -s:a compiler.cppstd=17
