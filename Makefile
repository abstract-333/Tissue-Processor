# Makefile — Arduino (arduino-cli)
# Edit these variables if needed
FQBN        ?= arduino:avr:uno
SKETCH      ?= TissueProcessor/TissueProcessor.ino
BUILD_DIR   ?= build
PORT        ?= COM3            # change to the serial port your Uno uses (e.g. COM3, /dev/ttyACM0)
ARDUINO_CLI ?= C:/Users/basha/AppData/Local/Programs/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe
# derived
ELF         := $(BUILD_DIR)/TissueProcessor.ino.elf
HEX         := $(BUILD_DIR)/TissueProcessor.ino.hex

.PHONY: all build compile upload clean size monitor

all: build

# compile/prepare build folder (creates .elf/.hex)
build:
	@echo "Compiling $(SKETCH) for $(FQBN)..."
	$(ARDUINO_CLI) compile --fqbn $(FQBN) --output-dir $(BUILD_DIR) $(SKETCH)

compile: build

# upload using arduino-cli (make sure PORT is set correctly)
upload: build
	@if [ -z "$(PORT)" ]; then echo "ERROR: set PORT variable (e.g. make upload PORT=COM3)"; exit 1; fi
	@echo "Uploading to $(PORT)..."
	$(ARDUINO_CLI) upload -p $(PORT) --fqbn $(FQBN) --input-dir $(BUILD_DIR)

# print program/ram usage (uses avr-size included with toolchain if available)
size: build
	@echo "Size info for $(ELF):"
	@if command -v avr-size >/dev/null 2>&1; then avr-size -A $(ELF); else echo "avr-size not found — arduino-cli compile output contains size info."; fi

# serial monitor (forward to arduino-cli monitor)
monitor:
	@if [ -z "$(PORT)" ]; then echo "ERROR: set PORT variable (e.g. make monitor PORT=COM3)"; exit 1; fi
	$(ARDUINO_CLI) monitor -p $(PORT)

# cleanup build artifacts
clean:
	@echo "Cleaning $(BUILD_DIR)..."
	@if [ -d "$(BUILD_DIR)" ]; then \
		if [ "$(OS)" = "Windows_NT" ]; then cmd /C rmdir /S /Q "$(BUILD_DIR)"; else rm -rf "$(BUILD_DIR)"; fi \
	else \
		echo "no build dir"; \
	fi

clint:
	uv run cpplint --filter=-legal/copyright --recursive tests/native/ TissueProcessor/


format: 
	clang-format -i TissueProcessor/*.ino tests/native/*.cpp