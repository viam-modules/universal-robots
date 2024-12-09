#!/usr/bin/env bash

set -eu

exec sudo nice -20 ./packaging/appimages/deploy/universal-robots-latest-x86_64.AppImage $@
