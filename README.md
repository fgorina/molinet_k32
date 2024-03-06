# Remote control of a windlass

This program runs in a ESP32 device (with WiFi and BLE) and it is designed to be connected
in parallel with the physical switches to the contactor unit.

Also the sensor in the windlass unit (of switch or similar type) and a corresponding
calibration) gives the length of chain.

The device is controlled via BLE and an application as [Milinet-K](https://github.com/fgorina/Molinet-K)
that works in iOS / AppleWatch or [Yamato_control](https://github.com/fgorina/Yamato_control) that is an extension
to [bbn_m5though_ative_boat](https://github.com/bareboat-necessities/bbn-m5stack-tough) that incorporates
the BLE connection to the windlass.

This program supports different commands for moving up/down a specified amount of chain. See the comments at the beginning or main.cpp
