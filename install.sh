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

# ---------------------------------------------------------------------------
# System setup (idempotent) — applies the steps from rpi_setup/setup.md so
# that a fresh Pi only needs the Raspberry Pi Imager flash + this script.
# Each step checks current state and only acts when something needs changing,
# so re-running this installer to update the binary stays fast.
# ---------------------------------------------------------------------------
REBOOT_NEEDED=0

if [ -d /boot/firmware ] && command -v raspi-config &> /dev/null; then
    echo -e "${GREEN}Running system setup checks...${NC}"

    # --- /boot/firmware/config.txt -----------------------------------------
    BOOT_CFG="/boot/firmware/config.txt"
    REPO_CFG_URL="https://raw.githubusercontent.com/$REPO/main/rpi_setup/config.txt"
    REPO_CFG_TMP="$(mktemp)"
    if curl -sSfL -o "$REPO_CFG_TMP" "$REPO_CFG_URL"; then
        if [ ! -f "$BOOT_CFG" ] || ! cmp -s "$REPO_CFG_TMP" "$BOOT_CFG"; then
            echo "  → Updating $BOOT_CFG"
            sudo cp "$REPO_CFG_TMP" "$BOOT_CFG"
            REBOOT_NEEDED=1
        else
            echo -e "  ${GREEN}✓${NC} $BOOT_CFG already up to date"
        fi
    else
        echo -e "  ${YELLOW}Warning: could not fetch $REPO_CFG_URL, skipping config.txt${NC}"
    fi
    rm -f "$REPO_CFG_TMP"

    # --- I2C ---------------------------------------------------------------
    if [ "$(sudo raspi-config nonint get_i2c)" != "0" ]; then
        echo "  → Enabling I2C"
        sudo raspi-config nonint do_i2c 0
        REBOOT_NEEDED=1
    else
        echo -e "  ${GREEN}✓${NC} I2C already enabled"
    fi

    # --- Serial: console OFF, hardware port ON -----------------------------
    # get_serial    → 1 means console disabled
    # get_serial_hw → 0 means hardware serial enabled
    if [ "$(sudo raspi-config nonint get_serial)" != "1" ] || \
       [ "$(sudo raspi-config nonint get_serial_hw)" != "0" ]; then
        echo "  → Disabling serial console, enabling serial port"
        sudo raspi-config nonint do_serial_cons 1 || true
        sudo raspi-config nonint do_serial_hw 0 || true
        REBOOT_NEEDED=1
    else
        echo -e "  ${GREEN}✓${NC} Serial already configured (console off, port on)"
    fi

    # --- Wifi powersave off ------------------------------------------------
    if command -v nmcli &> /dev/null; then
        WIFI_CON=$(nmcli -t -f NAME,TYPE con show --active | awk -F: '$2=="802-11-wireless"{print $1; exit}')
        if [ -n "$WIFI_CON" ]; then
            CUR_PS=$(nmcli -g 802-11-wireless.powersave con show "$WIFI_CON" 2>/dev/null || echo "")
            # 2 = disabled (per NetworkManager)
            if [ "$CUR_PS" != "2" ]; then
                echo "  → Disabling wifi powersave on '$WIFI_CON'"
                sudo nmcli con mod "$WIFI_CON" wifi.powersave 2
            else
                echo -e "  ${GREEN}✓${NC} Wifi powersave already disabled on '$WIFI_CON'"
            fi
        else
            echo -e "  ${YELLOW}Note: no active wifi connection found, skipping powersave${NC}"
        fi
    fi

    # --- Bluetooth: Privacy = device in main.conf --------------------------
    BT_CONF="/etc/bluetooth/main.conf"
    if [ -f "$BT_CONF" ]; then
        if grep -Eq '^\s*Privacy\s*=\s*device' "$BT_CONF"; then
            echo -e "  ${GREEN}✓${NC} Bluetooth Privacy already set"
        else
            echo "  → Setting Privacy = device in $BT_CONF"
            if grep -Eq '^\s*#?\s*Privacy\s*=' "$BT_CONF"; then
                sudo sed -i -E 's|^\s*#?\s*Privacy\s*=.*|Privacy = device|' "$BT_CONF"
            else
                if grep -q '^\[General\]' "$BT_CONF"; then
                    sudo sed -i '/^\[General\]/a Privacy = device' "$BT_CONF"
                else
                    echo -e "\n[General]\nPrivacy = device" | sudo tee -a "$BT_CONF" >/dev/null
                fi
            fi
            sudo systemctl restart bluetooth || true
        fi
    fi

    echo ""
fi

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

OPTIONAL_BINS="test_imu calibrate_imu test_i2c_raw test_controller init em check_voltage microduck_help debug_bno08x wheel_sysid reboot_motors la_poule"
for bin in $OPTIONAL_BINS; do
    if [ ! -f "$bin" ]; then
        echo -e "${YELLOW}Warning: $bin binary not found in archive.${NC}"
    fi
done

# Make binaries executable
chmod +x "$BINARY_NAME"
for bin in $OPTIONAL_BINS; do
    [ -f "$bin" ] && chmod +x "$bin"
done

# Install policies
if [ -d "policies" ]; then
    POLICIES_DIR="$HOME/microduck/policies"
    mkdir -p "$POLICIES_DIR"
    cp -a policies/. "$POLICIES_DIR/"
    echo -e "${GREEN}✓ Policies installed to $POLICIES_DIR${NC}"
fi

# Install URDF for inline odometry
if [ -f "robot.urdf" ]; then
    mkdir -p "$HOME/microduck"
    cp robot.urdf "$HOME/microduck/robot.urdf"
    echo -e "${GREEN}✓ URDF installed to $HOME/microduck/robot.urdf${NC}"
fi


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
    for bin in $OPTIONAL_BINS; do
        [ -f "$bin" ] && mv "$bin" "$INSTALL_DIR/"
    done
else
    echo "Installing with sudo (requires password)..."
    sudo mv "$BINARY_NAME" "$INSTALL_DIR/"
    for bin in $OPTIONAL_BINS; do
        [ -f "$bin" ] && sudo mv "$bin" "$INSTALL_DIR/"
    done
fi

# Ask which IMU is installed
echo ""
echo -e "${GREEN}Which IMU is installed on your robot?${NC}"
echo "  1) BMI088 (default)"
echo "  2) BNO08X (BNO080/085/086)"
echo "  3) BNO055"
read -r -p "Enter choice [1/2/3, default=1]: " IMU_CHOICE </dev/tty
case "$IMU_CHOICE" in
    2)
        IMU_FLAG="--bno08x"
        echo -e "  → Using ${GREEN}BNO08X${NC}"
        ;;
    3)
        IMU_FLAG="--bno055"
        echo -e "  → Using ${GREEN}BNO055${NC}"
        ;;
    *)
        IMU_FLAG=""
        echo -e "  → Using ${GREEN}BMI088${NC} (default)"
        ;;
esac
echo ""

# Ask whether to install the systemd service
SERVICE_NAME="microduck_runtime.service"
SERVICE_DIR="/etc/systemd/system"
if [ -f "$SERVICE_NAME" ]; then
    echo -e "${GREEN}Install systemd service?${NC}"
    echo "  This will start the runtime automatically on boot."
    read -r -p "Install service? [y/N, default=N]: " SERVICE_CHOICE </dev/tty
    case "$SERVICE_CHOICE" in
        y|Y)
            echo "Installing systemd service..."
            # Patch ExecStart with the IMU flag if needed
            if [ -n "$IMU_FLAG" ]; then
                sed -i "s|ExecStart=/usr/local/bin/microduck_runtime$|ExecStart=/usr/local/bin/microduck_runtime $IMU_FLAG|" "$SERVICE_NAME"
            fi
            sudo cp "$SERVICE_NAME" "$SERVICE_DIR/"
            sudo systemctl daemon-reload
            sudo systemctl enable "$SERVICE_NAME"
            echo -e "${GREEN}✓ Service installed and enabled (starts on boot)${NC}"
            echo "  Start now:  sudo systemctl start microduck_runtime"
            echo "  View logs:  journalctl -u microduck_runtime -f"
            ;;
        *)
            echo -e "  → ${YELLOW}Skipping service installation${NC}"
            ;;
    esac
else
    echo -e "${YELLOW}Note: $SERVICE_NAME not found in archive, skipping service setup.${NC}"
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
    echo "Run '$BINARY_NAME --help' for usage."
    if [ "$REBOOT_NEEDED" = "1" ]; then
        echo ""
        echo -e "${YELLOW}⚠ System settings changed — please reboot:${NC} sudo reboot"
    fi
else
    echo -e "${RED}Error: Installation failed.${NC}"
    echo "$BINARY_NAME not found in PATH."
    exit 1
fi
