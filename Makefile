## ------------------------------------------------------------------------- ##
##                                ENVIRONMENT SETUP                          ##
## ------------------------------------------------------------------------- ##

# Path to arduino-cli (using your specific path)
ARDUINO_CLI ?= C:/Users/basha/AppData/Local/Programs/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe

# Serial Port (Windows: COMx, Linux: /dev/ttyACMx)
PORT        ?= COM3

## -------------------------------------------------------------------------- ##
##                                 PROJECT SETUP                              ##
## -------------------------------------------------------------------------- ##

TARGET      := TissueProcessor
SKETCH      := $(TARGET)/$(TARGET).ino
FQBN        := arduino:avr:uno
BUILD_DIR   := build

# Derived paths for size checking
ELF         := $(BUILD_DIR)/$(TARGET).ino.elf

## -------------------------------------------------------------------------- ##
##                                BUILD TARGETS                               ##
## -------------------------------------------------------------------------- ##

.PHONY: all build compile upload clean size monitor lint format test

all: build

# Compile the sketch
build:
	@echo "--- Compiling $(SKETCH) for $(FQBN) ---"
	@if [ ! -d "$(BUILD_DIR)" ]; then mkdir -p $(BUILD_DIR); fi
	$(ARDUINO_CLI) compile --fqbn $(FQBN) --output-dir $(BUILD_DIR) $(SKETCH)

compile: build

# Upload to the board
upload: build
	@echo "--- Uploading to $(PORT) ---"
	$(ARDUINO_CLI) upload -p $(PORT) --fqbn $(FQBN) --input-dir $(BUILD_DIR)

# Show memory usage
size: build
	@echo "--- Size Info ---"
	$(ARDUINO_CLI) compile --fqbn $(FQBN) --show-properties $(SKETCH) | grep "recipe.size.pattern" || echo "Check build output for size."

# Open Serial Monitor
monitor:
	@echo "--- Opening Monitor on $(PORT) ---"
	$(ARDUINO_CLI) monitor -p $(PORT)

## -------------------------------------------------------------------------- ##
##                                MAINTENANCE                                 ##
## -------------------------------------------------------------------------- ##

# Cleanup build artifacts
clean:
	@echo "Cleaning $(BUILD_DIR)..."
	@if [ -d "$(BUILD_DIR)" ]; then \
		if [ "$(OS)" = "Windows_NT" ]; then cmd /C rmdir /S /Q "$(BUILD_DIR)"; else rm -rf "$(BUILD_DIR)"; fi \
	else \
		echo "Nothing to clean."; \
	fi

# Code Quality
lint:
	@echo "--- Running Linter ---"
	@if command -v uv >/dev/null 2>&1; then \
		uv run cpplint --filter=-legal/copyright --recursive tests/native/ $(TARGET)/; \
	else \
		cpplint --filter=-legal/copyright --recursive tests/native/ $(TARGET)/; \
	fi

format: 
	@echo "--- Formatting Code ---"
	clang-format -i $(TARGET)/*.ino tests/native/*.cpp

# Run Python tests
test: 
	@echo "--- Running PyTests ---"
	@if command -v uv >/dev/null 2>&1; then \
		uv run pytest tests/python; \
	else \
		pytest tests/python; \
	fi