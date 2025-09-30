#!/bin/bash
# set -e: exit with errors if anything fails
#     -u: it's an error to use an undefined variable
#     -x: print out every command before it runs
#     -o pipefail: if something in the middle of a pipeline fails, the whole thing fails
#
set -euxo pipefail

conan profile detect || echo "Conan is already installed"

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
# and in the Dockerfile
git checkout releases/v0.20.0

# Build the C++ SDK repo
#
# We want a static binary, so we turn off shared. Elect for C++17
# compilation, since it seems some of the dependencies we pick mandate
# it anyway.

# Create a profile file with version overrides
cat > protobuf-override.profile << 'EOF'
include(default)
[replace_tool_requires]
protobuf/*: protobuf/<host_version>
EOF

conan install . --update \
      --profile=protobuf-override.profile \
      --build={missing,outdated} \
      -s:a build_type=Release \
      -s:a compiler.cppstd=17 \
      -o:a "*:shared=False" \
      -o:a "&:shared=False"

conan create . \
      --profile=protobuf-override.profile \
      --build=viam-cpp-sdk/0.20.0 \
      -s:a build_type=Release \
      -s:a "&:build_type=RelWithDebInfo" \
      -s:a compiler.cppstd=17 \
      -o:a "*:shared=False" \
      -o:a "&:shared=False"

# Cleanup
popd  # viam-cpp-sdk
popd  # tmp_cpp_sdk
rm -rf tmp_cpp_sdk
