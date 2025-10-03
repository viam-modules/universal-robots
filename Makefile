.PHONY: \
	module.tar.gz \
	format \
	format-check \
	test \
	run-clang-tidy \
	run-clang-check \
	clean \
	clean-all \
	docker-amd64 \
	docker-arm64 \
	docker-build \
	docker-upload \
	docker \
	docker-arm64-ci \
	docker-amd64-ci

default: module.tar.gz

# format the source code
format:
	ls src/viam/ur/module/*.*pp | xargs clang-format-19 -i --style=file

format-check:
	ls src/viam/ur/module/*.*pp | xargs clang-format-19 -i --style=file --dry-run --Werror

configure:
	cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo -G Ninja

build: configure
	cmake --build build --target all -- -j4

install: build
	DESTDIR=build/install cmake --install build --prefix /

test: build
	./build/universal-robots-test

module.tar.gz: format-check install
	tar czf $@ -C build/install .

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

docker-amd64: MAIN_TAG = ghcr.io/viam-modules/universal-robots
docker-amd64: BUILD_TAG = amd64
docker-amd64:
	$(BUILD_CMD)

docker-arm64: MAIN_TAG = ghcr.io/viam-modules/universal-robots
docker-arm64: BUILD_TAG = arm64
docker-arm64:
	$(BUILD_CMD)

docker-build: docker-amd64 docker-arm64

docker-upload:
	docker push 'ghcr.io/viam-modules/universal-robots:amd64'
	docker push 'ghcr.io/viam-modules/universal-robots:arm64'

docker: docker-build docker-upload

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
