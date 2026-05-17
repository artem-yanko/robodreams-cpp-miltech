HW ?= homework_06
BUILD_PRESET ?= debug
BUILD_DIR := build/$(BUILD_PRESET)

FORMAT_FILES := $(shell find $(HW) \( -name '*.cpp' -o -name '*.hpp' \))
LINT_FILES := $(shell find $(HW) -name '*.cpp')

.PHONY: format lint test quality build run clean

format:
	clang-format -i $(FORMAT_FILES)

build:
	cmake --preset $(BUILD_PRESET)
	cmake --build --preset $(BUILD_PRESET)

test: build
	ctest --test-dir $(BUILD_DIR) --output-on-failure

lint: build
	clang-tidy $(LINT_FILES) -p $(BUILD_DIR) --quiet --header-filter='^.*$(HW)/.*'

quality: format test lint

run: build
	./$(BUILD_DIR)/$(HW)/ballistics_check $(HW)/data/sample_vog17.txt $(HW)/src/output.txt

clean:
	rm -rf build/$(BUILD_PRESET)