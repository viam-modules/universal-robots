#!/usr/bin/env bash

set -eu

exec sudo nice -20 ./universal-robots.AppImage $@

