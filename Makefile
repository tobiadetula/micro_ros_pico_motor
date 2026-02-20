.PHONY: build upload clean

build:
	rm -rf build
	mkdir -p build
	cd build && cmake .. && make -j4

upload:
	picotool reboot -f -u
	sleep 3
	cp build/ros_pico_motorcontrol.uf2 /media/$(USER)/RPI-RP2

all: build upload