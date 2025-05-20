#!/bin/bash
# set -e: exit with errors if anything fails
#     -u: it's an error to use an undefined variable
#     -x: print out every command before it runs
#     -o pipefail: if something in the middle of a pipeline fails, the whole thing fails
set -euxo pipefail

valgrind --error-exitcode=1 --leak-check=full --track-origins=yes --dsymutil=yes -v ./packaging/appimages/deploy/universal-robots-latest-x86_64.AppImage "$@"
