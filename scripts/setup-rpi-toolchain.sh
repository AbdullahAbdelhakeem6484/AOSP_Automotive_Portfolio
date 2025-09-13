#!/bin/bash

# AOSP Automotive Portfolio - Raspberry Pi 4 Toolchain Setup
# Author: Abdullah Abdelhakeem
# Email: abdullah.abdelhakeem657@gmail.com

set -e

echo "=== AOSP Automotive Portfolio - Raspberry Pi 4 Toolchain Setup ==="
echo "Setting up cross-compilation toolchain for Raspberry Pi 4..."

# Create toolchain directory
TOOLCHAIN_DIR="$HOME/rpi-toolchain"
mkdir -p "$TOOLCHAIN_DIR"

# Download Raspberry Pi cross-compilation toolchain
echo "Downloading Raspberry Pi toolchain..."
cd "$TOOLCHAIN_DIR"
if [ ! -d "tools" ]; then
    git clone https://github.com/raspberrypi/tools.git
fi

# Add toolchain to PATH
echo "Configuring environment..."
echo "export PATH=\$PATH:$TOOLCHAIN_DIR/tools/arm-bcm2708/arm-rpi-4.9.3-linux-gnueabihf/bin" >> ~/.bashrc

# Install required packages for AOSP build
echo "Installing AOSP build dependencies..."
sudo apt-get update
sudo apt-get install -y \
    git-core gnupg flex bison build-essential zip curl zlib1g-dev \
    gcc-multilib g++-multilib libc6-dev-i386 lib32ncurses5-dev \
    x11proto-core-dev libx11-dev lib32z1-dev libgl1-mesa-dev \
    libxml2-utils xsltproc unzip fontconfig python3-dev python3-pip

# Install Python dependencies for automotive projects
echo "Installing Python dependencies..."
pip3 install --user \
    opencv-python numpy scipy matplotlib \
    tensorflow scikit-learn \
    gps3 pyserial smbus2 \
    RPi.GPIO pigpio

# Install additional automotive-specific packages
sudo apt-get install -y \
    libopencv-dev python3-opencv \
    i2c-tools libi2c-dev \
    wiringpi

# Enable required interfaces on Raspberry Pi
echo "Configuring Raspberry Pi interfaces..."
if grep -q "dtparam=i2c_arm=on" /boot/config.txt; then
    echo "I2C already enabled"
else
    echo "dtparam=i2c_arm=on" | sudo tee -a /boot/config.txt
fi

if grep -q "dtparam=spi=on" /boot/config.txt; then
    echo "SPI already enabled"
else
    echo "dtparam=spi=on" | sudo tee -a /boot/config.txt
fi

if grep -q "dtoverlay=w1-gpio" /boot/config.txt; then
    echo "1-Wire already enabled"
else
    echo "dtoverlay=w1-gpio" | sudo tee -a /boot/config.txt
fi

# Add user to required groups
sudo usermod -a -G i2c,spi,gpio "$USER"

# Create project workspace
PROJECT_WORKSPACE="$HOME/aosp-automotive"
mkdir -p "$PROJECT_WORKSPACE"

echo "=== Setup Complete ==="
echo "Toolchain installed to: $TOOLCHAIN_DIR"
echo "Project workspace: $PROJECT_WORKSPACE"
echo ""
echo "IMPORTANT: Please reboot your Raspberry Pi to enable interface changes"
echo "After reboot, source your bashrc: source ~/.bashrc"
echo ""
echo "To verify setup, run:"
echo "  python3 -c 'import cv2, numpy, tensorflow; print(\"All dependencies installed successfully\")'"
echo "  gpio readall  # Should show GPIO pin status"
