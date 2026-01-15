use anyhow::Result;
use std::io::{Read, Write};
use std::fs::{File, OpenOptions};
use std::os::unix::io::AsRawFd;

const BNO055_I2C_ADDR: u16 = 0x28;
const BNO055_CHIP_ID_REG: u8 = 0x00;
const BNO055_EXPECTED_CHIP_ID: u8 = 0xA0;
const I2C_SLAVE: u16 = 0x0703;

fn main() -> Result<()> {
    println!("BNO055 Raw I2C Test");
    println!("===================\n");

    println!("Opening I2C device /dev/i2c-1...");
    let mut i2c = OpenOptions::new()
        .read(true)
        .write(true)
        .open("/dev/i2c-1")?;
    println!("✓ I2C device opened\n");

    // Set slave address
    unsafe {
        if libc::ioctl(i2c.as_raw_fd(), I2C_SLAVE as libc::c_ulong, BNO055_I2C_ADDR as libc::c_ulong) < 0 {
            return Err(anyhow::anyhow!("Failed to set I2C slave address"));
        }
    }
    println!("✓ I2C slave address set to 0x{:02X}\n", BNO055_I2C_ADDR);

    // Try to read chip ID
    println!("Reading BNO055 Chip ID (register 0x00)...");
    println!("Expected: 0xA0\n");

    // Write register address
    println!("Step 1: Writing register address...");
    match i2c.write(&[BNO055_CHIP_ID_REG]) {
        Ok(_) => println!("✓ Register address sent"),
        Err(e) => {
            println!("✗ Failed to write: {:?}", e);
            return Err(e.into());
        }
    }

    std::thread::sleep(std::time::Duration::from_millis(10));

    // Read chip ID
    let mut buffer = [0u8; 1];
    println!("Step 2: Reading chip ID byte...");
    match i2c.read(&mut buffer) {
        Ok(_) => {
            println!("✓ Successfully read from device");
            println!("\nChip ID: 0x{:02X}", buffer[0]);

            if buffer[0] == BNO055_EXPECTED_CHIP_ID {
                println!("✓ SUCCESS! Chip ID matches!");
                println!("  BNO055 is responding correctly.");
                println!("\nThe hardware is working. The bno055 crate initialization");
                println!("might have issues. Try updating the crate or checking");
                println!("if there are known issues with the BNO055 initialization sequence.");
            } else if buffer[0] == 0x00 || buffer[0] == 0xFF {
                println!("⚠ Got 0x{:02X} - device is not responding properly", buffer[0]);
                println!("  Possible causes:");
                println!("  - Device not powered");
                println!("  - Wrong I2C address (try 0x29 if ADR pin is high)");
                println!("  - Bus conflict");
            } else {
                println!("⚠ Chip ID mismatch!");
                println!("  Got 0x{:02X}, expected 0x{:02X}", buffer[0], BNO055_EXPECTED_CHIP_ID);
                println!("  This might not be a BNO055.");
            }
        }
        Err(e) => {
            println!("✗ Failed I2C communication: {:?}", e);
            println!("\nThis is an I2C communication error.");
            println!("Possible causes:");
            println!("  1. Wiring issue (check SDA on GPIO2, SCL on GPIO3)");
            println!("  2. Power issue (check VCC and GND)");
            println!("  3. Pull-up resistors missing (usually 4.7kΩ to 3.3V)");
            println!("  4. I2C speed too high - try lowering in /boot/firmware/config.txt:");
            println!("     dtparam=i2c_arm_baudrate=100000");
            return Err(e.into());
        }
    }

    Ok(())
}
