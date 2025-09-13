#!/bin/bash

# AOSP Automotive Portfolio - Development Environment Setup
# Author: Abdullah Abdelhakeem
# Email: abdullah.abdelhakeem657@gmail.com
# GitHub: @AbdullahAbdelhakeem6484

set -e

PROJECT_LEVEL=${1:-"beginner"}
ADVANCED_MODE=${2:-"false"}

echo "=== AOSP Automotive Portfolio - Environment Setup ==="
echo "Setting up development environment for $PROJECT_LEVEL level projects..."

# Create project directories if they don't exist
create_project_structure() {
    echo "Creating project directory structure..."
    
    mkdir -p 01-beginner/{automotive-dashboard-simulator,vehicle-state-monitor}
    mkdir -p 02-intermediate/{smart-climate-control,driver-assistance-alerts}
    mkdir -p 03-advanced/{autonomous-parking-assistant,intelligent-infotainment}
    mkdir -p 04-expert/{fleet-management-platform,automotive-security-framework}
    mkdir -p scripts docs hardware ml_models
    
    echo "✅ Project structure created"
}

# Install basic dependencies
install_basic_dependencies() {
    echo "Installing basic development dependencies..."
    
    # Update package manager
    sudo apt-get update
    
    # Install essential build tools
    sudo apt-get install -y \
        git curl wget vim nano \
        build-essential cmake \
        python3 python3-pip python3-dev \
        nodejs npm
    
    # Install basic Python packages
    pip3 install --user \
        numpy matplotlib scipy \
        requests json5 pyyaml
    
    echo "✅ Basic dependencies installed"
}

# Install AOSP build dependencies
install_aosp_dependencies() {
    echo "Installing AOSP build dependencies..."
    
    sudo apt-get install -y \
        git-core gnupg flex bison build-essential zip curl zlib1g-dev \
        gcc-multilib g++-multilib libc6-dev-i386 lib32ncurses5-dev \
        x11proto-core-dev libx11-dev lib32z1-dev libgl1-mesa-dev \
        libxml2-utils xsltproc unzip fontconfig
    
    echo "✅ AOSP dependencies installed"
}

# Install hardware interface libraries
install_hardware_dependencies() {
    echo "Installing hardware interface dependencies..."
    
    # GPIO and hardware control
    pip3 install --user \
        RPi.GPIO pigpio gpiozero \
        pyserial smbus2 spidev
    
    # Install hardware tools
    sudo apt-get install -y \
        i2c-tools libi2c-dev \
        wiringpi raspi-gpio
    
    echo "✅ Hardware dependencies installed"
}

# Install computer vision and ML dependencies
install_cv_ml_dependencies() {
    echo "Installing computer vision and machine learning dependencies..."
    
    # OpenCV and image processing
    sudo apt-get install -y \
        libopencv-dev python3-opencv \
        libgtk-3-dev libcanberra-gtk-module \
        libavcodec-dev libavformat-dev libswscale-dev
    
    # Machine learning frameworks
    pip3 install --user \
        opencv-python \
        tensorflow-lite-runtime \
        scikit-learn \
        pillow
    
    echo "✅ Computer vision and ML dependencies installed"
}

# Install advanced dependencies for expert projects
install_advanced_dependencies() {
    echo "Installing advanced dependencies..."
    
    # Robotics and autonomous systems
    pip3 install --user \
        scipy matplotlib \
        gps3 \
        pycryptodome \
        can-isotp python-can
    
    # Network and security tools
    sudo apt-get install -y \
        nmap wireshark-common \
        openssl libssl-dev \
        can-utils
    
    echo "✅ Advanced dependencies installed"
}

# Configure Raspberry Pi interfaces
configure_rpi_interfaces() {
    echo "Configuring Raspberry Pi interfaces..."
    
    # Enable I2C
    if ! grep -q "dtparam=i2c_arm=on" /boot/config.txt; then
        echo "dtparam=i2c_arm=on" | sudo tee -a /boot/config.txt
        echo "✅ I2C enabled"
    fi
    
    # Enable SPI
    if ! grep -q "dtparam=spi=on" /boot/config.txt; then
        echo "dtparam=spi=on" | sudo tee -a /boot/config.txt
        echo "✅ SPI enabled"
    fi
    
    # Enable Camera
    if ! grep -q "start_x=1" /boot/config.txt; then
        echo "start_x=1" | sudo tee -a /boot/config.txt
        echo "gpu_mem=128" | sudo tee -a /boot/config.txt
        echo "✅ Camera enabled"
    fi
    
    # Enable 1-Wire
    if ! grep -q "dtoverlay=w1-gpio" /boot/config.txt; then
        echo "dtoverlay=w1-gpio" | sudo tee -a /boot/config.txt
        echo "✅ 1-Wire enabled"
    fi
    
    # Add user to required groups
    sudo usermod -a -G i2c,spi,gpio,video "$USER"
    
    echo "⚠️  Reboot required to activate interface changes"
}

# Create development workspace
setup_workspace() {
    echo "Setting up development workspace..."
    
    # Create workspace directory
    WORKSPACE="$HOME/aosp-automotive-workspace"
    mkdir -p "$WORKSPACE"
    
    # Create symbolic link to project
    if [ ! -L "$WORKSPACE/portfolio" ]; then
        ln -s "$(pwd)" "$WORKSPACE/portfolio"
    fi
    
    # Create build directory
    mkdir -p "$WORKSPACE/build"
    mkdir -p "$WORKSPACE/logs"
    
    # Set environment variables
    echo "export AOSP_AUTOMOTIVE_WORKSPACE=$WORKSPACE" >> ~/.bashrc
    echo "export PATH=\$PATH:\$AOSP_AUTOMOTIVE_WORKSPACE/portfolio/scripts" >> ~/.bashrc
    
    echo "✅ Workspace created at $WORKSPACE"
}

# Install project-specific dependencies based on level
install_project_dependencies() {
    case $PROJECT_LEVEL in
        "beginner")
            echo "Installing beginner project dependencies..."
            pip3 install --user matplotlib seaborn
            ;;
        "intermediate")
            echo "Installing intermediate project dependencies..."
            install_cv_ml_dependencies
            ;;
        "advanced")
            echo "Installing advanced project dependencies..."
            install_cv_ml_dependencies
            install_advanced_dependencies
            ;;
        "expert")
            echo "Installing expert project dependencies..."
            install_cv_ml_dependencies
            install_advanced_dependencies
            # Additional expert-level tools
            pip3 install --user \
                cryptography \
                paramiko \
                flask django
            ;;
    esac
}

# Main setup execution
main() {
    echo "Starting setup for $PROJECT_LEVEL level projects..."
    
    create_project_structure
    install_basic_dependencies
    install_aosp_dependencies
    install_hardware_dependencies
    install_project_dependencies
    
    if [ "$ADVANCED_MODE" = "true" ] || [ "$PROJECT_LEVEL" = "advanced" ] || [ "$PROJECT_LEVEL" = "expert" ]; then
        install_advanced_dependencies
    fi
    
    configure_rpi_interfaces
    setup_workspace
    
    echo ""
    echo "=== Setup Complete! ==="
    echo "✅ Development environment ready for $PROJECT_LEVEL level projects"
    echo "✅ Project structure created"
    echo "✅ Dependencies installed"
    echo "✅ Raspberry Pi interfaces configured"
    echo "✅ Development workspace ready"
    echo ""
    echo "Next steps:"
    echo "1. Reboot your Raspberry Pi: sudo reboot"
    echo "2. Source your bashrc: source ~/.bashrc"
    echo "3. Navigate to your first project:"
    echo "   cd 01-beginner/automotive-dashboard-simulator"
    echo "4. Follow the project README for specific setup instructions"
    echo ""
    echo "For support: abdullah.abdelhakeem657@gmail.com"
    echo "GitHub: https://github.com/AbdullahAbdelhakeem6484"
}

# Run main setup
main "$@"
