BUILD_DIR ?= build
CMAKE ?= cmake

$(BUILD_DIR)/universal-robots:
	rm -rf build
	$(CMAKE) -G Ninja -B $(BUILD_DIR) .
	$(CMAKE) --build $(BUILD_DIR) --target universal-robots

setup:
	sudo apt-get update
	sudo apt-get install -qqy libeigen3-dev