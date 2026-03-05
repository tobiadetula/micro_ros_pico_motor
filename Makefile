.PHONY: build upload clean all help build-libs rebuild
.DEFAULT_GOAL := help

# Config
BUILD_DIR   := build
LIB_DIR     := libs
SRC_DIR     := src
TARGET      := ros_pico_motorcontrol
MOUNT_POINT := /media/$(USER)/RPI-RP2
LOG_FILE    := build.log
SLEEP_TIME  := 5

help:
	@echo ""
	@echo "Available targets:"
	@echo "  build        - Configure and build firmware"
	@echo "  build-libs   - Build pico-servo and dependencies"
	@echo "  rebuild      - Rebuild without cleaning"
	@echo "  upload       - Upload firmware to Pico"
	@echo "  clean        - Remove build artifacts"
	@echo "  all          - Build and upload"
	@echo ""
build:
	@echo "==> Cleaning build directory..."
	@rm -rf $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)
	@echo "==> Running cmake..."
	@cd $(BUILD_DIR) && \
	bash -c 'set -o pipefail && cmake .. 2>&1 | tee ../$(LOG_FILE)' || \
		(echo "==> BUILD FAILED (cmake). Check $(LOG_FILE)." && exit 1)
	@echo "==> Building..."
	@cd $(BUILD_DIR) && \
	bash -c 'set -o pipefail && make -j4 2>&1 | tee -a ../$(LOG_FILE)' || \
		(echo "==> BUILD FAILED (make). Check $(LOG_FILE)." && exit 1)
	@echo "==> Build successful: $(BUILD_DIR)/$(TARGET).uf2"

build-libs:
	@echo "==> Building pico-servo and dependencies..."
	@cd $(LIB_DIR)/pico-servo && \
		git submodule update --init --recursive 2>&1 | tee -a ../$(LOG_FILE) || \
		(echo "ERROR: submodule init failed." && exit 1)
	@cd $(LIB_DIR)/pico-servo/build && \
		cmake .. 2>&1 | tee -a ../$(LOG_FILE) && \
		make -j4 2>&1 | tee -a ../$(LOG_FILE) || \
		(echo "ERROR: pico-servo build failed. Check $(LOG_FILE)." && exit 1)
# 	@cd $(LIB_DIR)/pico-ina219 && \
# 		cmake .. 2>&1 | tee -a ../$(LOG_FILE) && \
# 		make -j4 2>&1 | tee -a ../$(LOG_FILE) || \
# 		(echo "ERROR: pico-ina219 build failed. Check $(LOG_FILE)." && exit 1)
	@echo "==> Libraries built successfully."

rebuild:
	@cd $(BUILD_DIR) && \
	bash -c 'set -o pipefail && make -j4 2>&1 | tee -a ../$(LOG_FILE)'
	@if [ $$? -ne 0 ]; then \
		echo "ERROR: make failed."; \
		exit 1; \
	fi
	@echo "==> Rebuild successful."

upload:
	@echo "==> Checking .uf2 exists..."
	@test -f $(BUILD_DIR)/$(TARGET).uf2 || \
		(echo "ERROR: $(TARGET).uf2 not found. Run 'make build' first." && exit 1)
	@echo "==> Rebooting Pico into BOOTSEL mode..."
	@picotool reboot -f -u 2>&1 || \
		(echo "ERROR: picotool failed. Is the Pico connected?" && exit 1)
	@echo "==> Waiting for Pico to mount..."
	@sleep $(SLEEP_TIME)
	@test -d $(MOUNT_POINT) || \
		(echo "ERROR: $(MOUNT_POINT) not found. Try increasing SLEEP_TIME." && exit 1)
	@echo "==> Copying firmware..."
	@cp $(BUILD_DIR)/$(TARGET).uf2 $(MOUNT_POINT) || \
		(echo "ERROR: Failed to copy .uf2 to $(MOUNT_POINT)." && exit 1)
	@echo "==> Upload successful. Pico is rebooting..."

clean:
	@echo "==> Cleaning build directory..."
	@rm -rf $(BUILD_DIR) $(LOG_FILE)
	@echo "==> Done."

all: build upload