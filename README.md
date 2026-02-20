# micro_ros_pico
Code for micro ros on rp2040 pico for masters thesis 

## Setup
git clone --recurse-submodules https://github.com/YOUR_USER/ros_pico_motor.git
cd ros_pico_motor

## Build
make build

## Upload
make upload
```

This documents the exact steps so you or anyone else can get going on a fresh machine without guessing.

---

Your final structure should look like:
```
ros_pico_motor/
├── pico-sdk/          # submodule - raspberrypi/pico-sdk
├── sdk/               # submodule - your micro-ROS fork
├── src/
│   └── main.c
├── examples/
│   └── basic_publisher/
│       └── main.c
├── .gitignore
├── .gitmodules
├── CMakeLists.txt
├── CMakePresets.json
├── Makefile
└── README.md