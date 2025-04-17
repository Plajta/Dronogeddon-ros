# Firmware

Firmware for the TELLO RMTT expansion featuring three **VL53L1X**, one **VL53L3CX**, and one **ToF Mini Lidar**.

### Used Libraries

We use `VL53L1X.h` and `SparkFun_VL53L5CX_Library.h`, in addition to all dependencies required by the default RMTT code.

### Ways of Uploading

- **Method 1:** Use the Chinese **Mind+** program and its **Upload Code** feature. This is also where you can find the default code for the RMTT module.

- **Method 2 (recommended):** Use [this GitHub repository](https://github.com/MCTRACO/tellotalent_arduino_package) and set up your **Arduino IDE** with the proper settings for the ESP32 chip in the RMTT module.
