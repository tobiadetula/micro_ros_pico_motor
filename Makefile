.PHONY: build upload clean all

# Config
BUILD_DIR   := build
TARGET      := ros_pico_motorcontrol
MOUNT_POINT := /media/$(USER)/RPI-RP2
LOG_FILE    := build.log
SLEEP_TIME  := 3

build:
	@echo "==> Cleaning build directory..."
	@rm -rf $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)
	@echo "==> Running cmake..."
	@cd $(BUILD_DIR) && cmake .. 2>&1 | tee ../$(LOG_FILE) || \
		(echo "ERROR: cmake failed. Check $(LOG_FILE) for details." && exit 1)
	@echo "==> Building..."
	@cd $(BUILD_DIR) && make -j4 2>&1 | tee -a ../$(LOG_FILE) || \
		(echo "ERROR: make failed. Check $(LOG_FILE) for details." && exit 1)
	@echo "==> Build successful: $(BUILD_DIR)/$(TARGET).uf2"

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