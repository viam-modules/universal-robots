default: build/universal-robots

# format the source code
format:
	ls src/*.*pp main.cpp | xargs clang-format-15 -i --style=file

build: 
	mkdir build

SANITIZE ?= OFF
build/universal-robots: format build
	cd build && \
	cmake -G Ninja -DENABLE_SANITIZER=$(SANITIZE) .. && \
	ninja all -j 4

clean: 
	rm -rf build

clean-all:
	git clean -fxd

# Docker
BUILD_CMD = docker buildx build --pull $(BUILD_PUSH) --force-rm --build-arg MAIN_TAG=$(MAIN_TAG) \
	--build-arg BASE_TAG=$(BUILD_TAG) --platform linux/$(BUILD_TAG) -f $(BUILD_FILE) -t '$(MAIN_TAG):$(BUILD_TAG)' .
BUILD_PUSH = --load
BUILD_FILE = Dockerfile

docker: docker-build docker-upload

docker-build: docker-amd64

docker-amd64: MAIN_TAG = ghcr.io/viam-modules/universal-robots
docker-amd64: BUILD_TAG = amd64
docker-amd64:
	$(BUILD_CMD)

docker-upload:
	docker push 'ghcr.io/viam-modules/universal-robots:amd64'

# CI targets that automatically push, avoid for local test-first-then-push workflows
docker-arm64-ci: MAIN_TAG = ghcr.io/viam-modules/universal-robots
docker-arm64-ci: BUILD_TAG = arm64
docker-arm64-ci: BUILD_PUSH = --push
docker-arm64-ci:
	$(BUILD_CMD)

docker-amd64-ci: MAIN_TAG = ghcr.io/viam-modules/universal-robots
docker-amd64-ci: BUILD_TAG = amd64
docker-amd64-ci: BUILD_PUSH = --push
docker-amd64-ci:
	$(BUILD_CMD)

# Define a function for building AppImages
TAG_VERSION?=latest
define BUILD_APPIMAGE
    export TAG_NAME=$(TAG_VERSION); \
    cd packaging/appimages && \
    mkdir -p deploy && \
    rm -f deploy/$(1)* && \
    appimage-builder --recipe $(1)-$(2).yml
endef

# Targets for building AppImages
appimage-arm64: export OUTPUT_NAME = universal-robots
appimage-arm64: export ARCH = aarch64
appimage-arm64: build/universal-robots
	$(call BUILD_APPIMAGE,$(OUTPUT_NAME),$(ARCH))
	mv ./packaging/appimages/$(OUTPUT_NAME)-*-$(ARCH).AppImage* ./packaging/appimages/deploy/

appimage-amd64: export OUTPUT_NAME = universal-robots
appimage-amd64: export ARCH = x86_64
appimage-amd64: build/universal-robots
	$(call BUILD_APPIMAGE,$(OUTPUT_NAME),$(ARCH))
	mv ./packaging/appimages/$(OUTPUT_NAME)-*-$(ARCH).AppImage* ./packaging/appimages/deploy/

appimages: appimage-amd64 appimage-arm64
