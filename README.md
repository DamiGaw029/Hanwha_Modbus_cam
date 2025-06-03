# Vision System for Hanwha HCR-3 with Basler Camera and Modbus TCP

This project provides a real-time image processing system for integration with the **Hanwha HCR-3 collaborative robot**. It uses a **Basler industrial camera** to detect geometric features (circles or QR codes) and sends measurement results to the robot via **Modbus TCP**.

## Key Features

- Real-time detection of:
  - Circles using OpenCV's Hough Transform
  - QR codes (position, size, content) using OpenCV
- Pixel measurements converted to millimeters
- Coordinates reported relative to image center
- Communication with **Hanwha HCR-3 robot** via **Modbus TCP**
- Fully configurable via Modbus registers

## System Overview

- **Basler camera** captures and processes image data
- **Python application** analyzes frames for shapes or codes
- **Modbus TCP server** provides interface to Hanwha cobot
- Hanwha HCR-3 reads data such as object position, size, and QR contents for downstream actions (e.g., pick and place)

## Requirements

- Python 3.7+
- [Basler Pylon SDK](https://www.baslerweb.com/en/products/software/basler-pylon-camera-software-suite/)
- Python packages:
  ```bash
  pip install opencv-python numpy pymodbus pypylon
  
# How to use

Run main.py
