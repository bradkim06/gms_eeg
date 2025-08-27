# GMS_EEG Firmware

This project contains the firmware for the GMS_EEG device. It is built using the nRF Connect SDK and Zephyr RTOS.

## Overview

The primary function of this firmware is to act as a bridge between a UART-connected sensor and a Bluetooth Low Energy (BLE) central device.

It continuously listens on the UART interface for incoming byte streams. It then parses these streams to identify and assemble 17-byte data packets defined by the HHS protocol (identified by the `A5 5A 02` header).

Once complete HHS packets are received, they are buffered and transmitted wirelessly over BLE using the Nordic UART Service (NUS). The firmware is optimized to batch multiple HHS packets together to make efficient use of the BLE connection's Maximum Transmission Unit (MTU).

## Key Technologies

- **Framework:** [Zephyr RTOS](https://www.zephyrproject.org/)
- **SDK:** [Nordic nRF Connect SDK](https://www.nordicsemi.com/Software-and-tools/Software/nRF-Connect-SDK)
- **Wireless Protocol:** Bluetooth Low Energy (BLE)
- **BLE Service:** Nordic UART Service (NUS)

## Hardware

This firmware is designed to run on custom hardware based on the Nordic nRF5340 SoC. The specific board target is `gms/nrf5340/cpuapp`.

## Building and Running

This project is configured to be built with the standard Zephyr/nRF Connect SDK toolchain.

1.  **Set up the nRF Connect SDK:** Follow the official Nordic Semiconductor documentation to install the toolchain and dependencies.

2.  **Build the application:** Use the `west` command to build the firmware for the target board.

    ```sh
    west build -b gms/nrf5340/cpuapp
    ```

3.  **Flash the device:** Once the build is complete, flash the resulting firmware to the device.

    ```sh
    west flash
    ```

After flashing, the device will begin advertising as `GMS_EEG` (or as configured in `prj.conf`). You can connect to it using a BLE client that supports the NUS service to receive the HHS packet data.
