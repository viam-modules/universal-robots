#!/bin/bash
# set -e: exit with errors if anything fails
#     -u: it's an error to use an undefined variable
#     -x: print out every command before it runs
#     -o pipefail: if something in the middle of a pipeline fails, the whole thing fails
#
set -euxo pipefail

OS=$(uname -s | tr '[:upper:]' '[:lower:]')

if [[ ${OS} == "darwin" ]]; then
    echo "Detected MacOS"

  if ! command -v brew >/dev/null; then
     echo "Brew not installed. Please install brew!"
     exit 1
  fi
  # Install required tools
  brew install cmake python@3.11 wget unzip || true
elif  [[ ${OS} == "linux" ]]; then
    echo "Detected Linux"
    # NOTE: this is written under the assumption that it will be built in canon
    sudo apt -y update && sudo apt -y upgrade && sudo apt install -y cmake python3.11 python3.11-venv wget
else
    echo "Unsupported OS: ${OS}"
    exit 1
fi

if [ ! -f "./venv/bin/activate" ]; then
  echo 'creating and sourcing virtual env'
  python3 -m venv venv && source ./venv/bin/activate
else
  echo 'sourcing virtual env'
  source ./venv/bin/activate
fi

# Set up conan
if [ ! -f "./venv/bin/conan" ]; then
  echo 'installing conan'
  python3 -m pip install conan
fi

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
git checkout releases/v0.19.0

# Build the C++ SDK repo
#
# We want a static binary, so we turn off shared. Elect for C++17
# compilation, since it seems some of the dependencies we pick mandate
# it anyway.
#
#       -o:a "&:shared=False" \
    #

# Create a profile file with version overrides
cat > protobuf-override.profile << 'EOF'
include(default)

[replace_requires]
protobuf/*: protobuf/5.27.0

[replace_tool_requires]
protobuf/*: protobuf/5.27.0
EOF


conan install . --update \
      --profile=protobuf-override.profile \
      --build={missing,outdated} \
      -s:a build_type=Release \
      -s:a compiler.cppstd=17

conan create . \
      --profile=protobuf-override.profile \
      --build=viam-cpp-sdk/0.19.0 \
      -s:a build_type=Release \
      -s:a "&:build_type=RelWithDebInfo" \
      -s:a compiler.cppstd=17

# Cleanup
popd  # viam-cpp-sdk
popd  # tmp_cpp_sdk
rm -rf tmp_cpp_sdk
