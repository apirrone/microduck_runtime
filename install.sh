#!/bin/bash

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
# Set via environment variable or use default
REPO="${MICRODUCK_REPO:-apirrone/microduck_runtime}"
BINARY_NAME="microduck_runtime"
INSTALL_DIR="/usr/local/bin"

echo -e "${GREEN}╔════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║   Microduck Runtime Installer         ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════╝${NC}"
echo ""

# Detect architecture
ARCH=$(uname -m)
case $ARCH in
    aarch64|arm64)
        RELEASE_ARCH="aarch64"
        ;;
    armv7l|armv7)
        echo -e "${YELLOW}Warning: ARMv7 detected. This installer targets ARM64.${NC}"
        echo -e "${YELLOW}The binary may not work on 32-bit Raspberry Pi OS.${NC}"
        echo -e "${YELLOW}Consider using Raspberry Pi OS 64-bit.${NC}"
        RELEASE_ARCH="aarch64"
        ;;
    x86_64)
        echo -e "${RED}Error: x86_64 architecture detected.${NC}"
        echo -e "${RED}This tool is designed for Raspberry Pi (ARM).${NC}"
        echo -e "${RED}Please compile from source if you need x86_64 support.${NC}"
        exit 1
        ;;
    *)
        echo -e "${RED}Error: Unsupported architecture: $ARCH${NC}"
        exit 1
        ;;
esac

echo -e "Detected architecture: ${GREEN}$ARCH${NC}"
echo ""

# Get latest release
echo "Fetching latest release..."
LATEST_RELEASE=$(curl -s https://api.github.com/repos/$REPO/releases/latest | grep "tag_name" | cut -d '"' -f 4)

if [ -z "$LATEST_RELEASE" ]; then
    echo -e "${RED}Error: Could not fetch latest release.${NC}"
    echo "Please check:"
    echo "  1. Repository exists: https://github.com/$REPO"
    echo "  2. There is at least one release"
    echo "  3. You have internet connectivity"
    exit 1
fi

echo -e "Latest release: ${GREEN}$LATEST_RELEASE${NC}"
echo ""

# Download URL
DOWNLOAD_URL="https://github.com/$REPO/releases/download/$LATEST_RELEASE/${BINARY_NAME}-${RELEASE_ARCH}-linux.tar.gz"

# Create temporary directory
TMP_DIR=$(mktemp -d)
cd "$TMP_DIR"

echo "Downloading $BINARY_NAME..."
if ! curl -L -o "${BINARY_NAME}.tar.gz" "$DOWNLOAD_URL"; then
    echo -e "${RED}Error: Download failed.${NC}"
    echo "URL: $DOWNLOAD_URL"
    rm -rf "$TMP_DIR"
    exit 1
fi

echo "Extracting archive..."
tar xzf "${BINARY_NAME}.tar.gz"

if [ ! -f "$BINARY_NAME" ]; then
    echo -e "${RED}Error: Binary not found in archive.${NC}"
    rm -rf "$TMP_DIR"
    exit 1
fi

if [ ! -f "test_imu" ]; then
    echo -e "${YELLOW}Warning: test_imu binary not found in archive.${NC}"
fi

if [ ! -f "test_imu2" ]; then
    echo -e "${YELLOW}Warning: test_imu2 binary not found in archive.${NC}"
fi

if [ ! -f "test_imu3" ]; then
    echo -e "${YELLOW}Warning: test_imu3 binary not found in archive.${NC}"
fi

if [ ! -f "debug_imu" ]; then
    echo -e "${YELLOW}Warning: debug_imu binary not found in archive.${NC}"
fi

if [ ! -f "debug_imu_observations" ]; then
    echo -e "${YELLOW}Warning: debug_imu_observations binary not found in archive.${NC}"
fi

if [ ! -f "test_i2c_raw" ]; then
    echo -e "${YELLOW}Warning: test_i2c_raw binary not found in archive.${NC}"
fi

# Make binaries executable
chmod +x "$BINARY_NAME"
[ -f "test_imu" ] && chmod +x "test_imu"
[ -f "test_imu2" ] && chmod +x "test_imu2"
[ -f "test_imu3" ] && chmod +x "test_imu3"
[ -f "debug_imu" ] && chmod +x "debug_imu"
[ -f "debug_imu_observations" ] && chmod +x "debug_imu_observations"
[ -f "test_i2c_raw" ] && chmod +x "test_i2c_raw"

# Install ONNX Runtime library
ONNX_LIB_DIR="/usr/local/lib"
if ls libonnxruntime.so* 1> /dev/null 2>&1; then
    echo "Installing ONNX Runtime library..."
    if [ -w "$ONNX_LIB_DIR" ]; then
        cp -a libonnxruntime.so* "$ONNX_LIB_DIR/"
    else
        sudo cp -a libonnxruntime.so* "$ONNX_LIB_DIR/"
    fi
    # Update library cache
    if command -v ldconfig &> /dev/null; then
        sudo ldconfig
    fi
fi

# Install binaries
echo "Installing to $INSTALL_DIR..."
if [ -w "$INSTALL_DIR" ]; then
    mv "$BINARY_NAME" "$INSTALL_DIR/"
    [ -f "test_imu" ] && mv "test_imu" "$INSTALL_DIR/"
    [ -f "test_imu2" ] && mv "test_imu2" "$INSTALL_DIR/"
    [ -f "test_imu3" ] && mv "test_imu3" "$INSTALL_DIR/"
    [ -f "debug_imu" ] && mv "debug_imu" "$INSTALL_DIR/"
    [ -f "debug_imu_observations" ] && mv "debug_imu_observations" "$INSTALL_DIR/"
    [ -f "test_i2c_raw" ] && mv "test_i2c_raw" "$INSTALL_DIR/"
else
    echo "Installing with sudo (requires password)..."
    sudo mv "$BINARY_NAME" "$INSTALL_DIR/"
    [ -f "test_imu" ] && sudo mv "test_imu" "$INSTALL_DIR/"
    [ -f "test_imu2" ] && sudo mv "test_imu2" "$INSTALL_DIR/"
    [ -f "test_imu3" ] && sudo mv "test_imu3" "$INSTALL_DIR/"
    [ -f "debug_imu" ] && sudo mv "debug_imu" "$INSTALL_DIR/"
    [ -f "debug_imu_observations" ] && sudo mv "debug_imu_observations" "$INSTALL_DIR/"
    [ -f "test_i2c_raw" ] && sudo mv "test_i2c_raw" "$INSTALL_DIR/"
fi

# Cleanup
cd ~
rm -rf "$TMP_DIR"

# Verify installation
if command -v $BINARY_NAME &> /dev/null; then
    VERSION=$($BINARY_NAME --version 2>&1 || echo "unknown")
    echo ""
    echo -e "${GREEN}✓ Installation successful!${NC}"
    echo -e "Version: ${GREEN}$VERSION${NC}"
    echo ""
    echo "Installed binaries:"
    echo "  - $BINARY_NAME (main runtime)"
    if command -v test_imu &> /dev/null; then
        echo "  - test_imu (IMU testing tool)"
    fi
    if command -v test_imu2 &> /dev/null; then
        echo "  - test_imu2 (simple axis remapping test)"
    fi
    if command -v test_imu3 &> /dev/null; then
        echo "  - test_imu3 (hardware axis remapping test)"
    fi
    if command -v debug_imu &> /dev/null; then
        echo "  - debug_imu (detailed IMU debugging tool)"
    fi
    if command -v debug_imu_observations &> /dev/null; then
        echo "  - debug_imu_observations (policy observations monitor)"
    fi
    if command -v test_i2c_raw &> /dev/null; then
        echo "  - test_i2c_raw (I2C diagnostic tool)"
    fi
    echo ""
    echo "Usage:"
    echo "  $BINARY_NAME --help"
    echo "  $BINARY_NAME --dummy"
    echo "  test_imu                    # Test BNO055 IMU (basic)"
    echo "  test_imu2                   # Simple axis remapping verification"
    echo "  test_imu3                   # Hardware-based axis remapping (bno055 crate)"
    echo "  debug_imu                   # Debug IMU with Euler angles"
    echo "  debug_imu_observations      # Monitor exact policy observations"
    echo "  test_i2c_raw                # Raw I2C diagnostic"
    echo ""
    echo "Example:"
    echo "  $BINARY_NAME --dummy --port /dev/ttyAMA0 --freq 50 --kp 400"
else
    echo -e "${RED}Error: Installation failed.${NC}"
    echo "$BINARY_NAME not found in PATH."
    exit 1
fi
