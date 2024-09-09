find ./src -type f \( -name \*.cpp -o -name \*.hpp \) | xargs clang-format -style=file -i -fallback-style=none "$@"
find ./src -type f \( -name \*.cpp -o -name \*.hpp \) | xargs clang-tidy "$@"