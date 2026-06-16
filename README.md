# Directional Antenna Controller for UAV Data Link

## Project Overview

This project presents a directional antenna controller designed to improve UAV communication reliability and link performance during flight operations.

The system combines an ESP32-S3 embedded controller, GPS positioning, electronic compass measurements, altitude sensing, and software-defined radio technologies to dynamically support directional communication experiments and antenna sector selection.

## Objectives

* Improve UAV communication reliability.
* Support directional antenna operation.
* Evaluate RF link performance.
* Integrate embedded sensing and positioning data.
* Demonstrate SDR-based transmission and reception experiments.

## System Components

### Embedded Controller

* ESP32-S3
* GPS Module
* MMC5983MA Electronic Compass
* BMP390 Altitude Sensor
* Bluetooth Low Energy Interface

### Software Defined Radio

* GNU Radio
* bladeRF SDR
* Custom transmit and receive flowgraphs

## Repository Structure

```text
firmware/
    ESP32_Code.ino

gnuradio/
    tx_sine.grc
    tx_sine.py
    rx_sine.grc
    rx_sine.py
    rx_sine_epy_block_0.py

docs/

images/
```

## Technologies

* ESP32-S3
* GNU Radio
* Python
* SDR
* GPS
* BLE
* Embedded Systems

## Author

Abdulrahman Mansour

Computer Engineer
