.PHONY: module.tar.gz ur5e-sim format format-check test run-clang-tidy run-clang-check clean clean-all docker docker-build docker-amd64 docker-upload appimages docker-arm64-ci docker-amd64-ci
default: build

# format the source code
format:
	ls src/viam/ur/module/*.*pp | xargs clang-format-19 -i --style=file

format-check:
	ls src/viam/ur/module/*.*pp | xargs clang-format-19 -i --style=file --dry-run --Werror

configure:
	cmake -S . -B build -G Ninja

build: configure
	cmake --build build --target all -- -j4
	cmake --install build --prefix build/install/usr

test: build
	./build/universal-robots-test

run-clang-tidy:
	clang-tidy-19 \
        -p build \
        --config-file ./.clang-tidy \
        --header-filter=".*/viam/ur/module/.*" \
	./src/viam/ur/module/*.cpp


run-clang-check:
	clang-check-19 -p build ./src/viam/ur/module/*.cpp

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
    rm -f deploy/*$(2)* && \
    appimage-builder --recipe $(1)-$(2).yml
endef

# Targets for building AppImages
appimage-arm64: export OUTPUT_NAME = universal-robots
appimage-arm64: export ARCH = aarch64
appimage-arm64: format build
	$(call BUILD_APPIMAGE,$(OUTPUT_NAME),$(ARCH))
	mv ./packaging/appimages/$(OUTPUT_NAME)-*-$(ARCH).AppImage* ./packaging/appimages/deploy/

appimage-amd64: export OUTPUT_NAME = universal-robots
appimage-amd64: export ARCH = x86_64
appimage-amd64: format build
	$(call BUILD_APPIMAGE,$(OUTPUT_NAME),$(ARCH))
	mv ./packaging/appimages/$(OUTPUT_NAME)-*-$(ARCH).AppImage* ./packaging/appimages/deploy/

appimages: appimage-amd64 appimage-arm64

module.tar.gz: meta.json
	cp ./packaging/appimages/deploy/universal-robots-latest-$(ARCH).AppImage universal-robots.AppImage
	tar czf $@ $^ universal-robots.AppImage

build/_deps/universal_robots_client_library-src/scripts/start_ursim.sh: build
	# we need to ignore `cmake -G Ninja  ..` failing as the this project's CMakeLists.txt
	# assumes that the viam sdk is available as a system package (which won't be true on most
	# development machines).
	# However even though it fails, it will still download theuniversal-robots SDK repo which
	# is all we need to run the sim.
	cd build && \
	cmake -G Ninja  ..  || true

ur3e-sim: build/_deps/universal_robots_client_library-src/scripts/start_ursim.sh
	build/_deps/universal_robots_client_library-src/scripts/start_ursim.sh -m ur3e 5.9.4 -p tests/resources/dockerursim/programs/e-serieuniversal_robots_client_library-srcs

ur5e-sim: build/_deps/universal_robots_client_library-src/scripts/start_ursim.sh
	build/_deps/universal_robots_client_library-src/scripts/start_ursim.sh -m ur5e 5.9.4 -p tests/resources/dockerursim/programs/e-serieuniversal_robots_client_library-srcs

ur20-sim: build/_deps/universal_robots_client_library-src/scripts/start_ursim.sh
	build/_deps/universal_robots_client_library-src/scripts/start_ursim.sh -m ur20 latest -p tests/resources/dockerursim/programs/e-serieuniversal_robots_client_library-srcs
