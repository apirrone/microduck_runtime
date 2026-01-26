# Changing Baudrate to 2M

## Current Status
- **Current baudrate**: 1,000,000 (1M)
- **Proposed baudrate**: 2,000,000 (2M)
- **Potential speedup**: ~1.15ms per control loop

## Prerequisites
✅ RPi Zero 2W supports 2M baud (PL011 UART)
✅ XL330 motors support up to 4.5M baud
⚠️  Requires good quality cables (short, shielded preferred)

## Step 1: Change Motor Baudrate (ONE TIME SETUP)

You need to reconfigure each motor to use 2M baud. Create a script:

```python
#!/usr/bin/env python3
# save as: set_baudrate_2m.py

from rustypot.xl330 import Xl330PyController
import time

# Motor IDs
MOTOR_IDS = [20, 21, 22, 23, 24, 30, 31, 32, 33, 10, 11, 12, 13, 14]

# Connect at current baudrate (1M)
controller = Xl330PyController("/dev/ttyUSB0", 1_000_000, 0.1)

print("Changing all motors to 2M baudrate...")
for motor_id in MOTOR_IDS:
    try:
        # Baudrate value 4 = 2M (see XL330 manual)
        # Register 8 = baud_rate
        controller.write_raw_data(motor_id, 8, [4])
        print(f"✓ Motor {motor_id} set to 2M")
        time.sleep(0.1)
    except Exception as e:
        print(f"✗ Motor {motor_id} failed: {e}")

print("\nDone! Power cycle the motors to apply changes.")
print("Then update microduck_runtime to use 2M baudrate.")
```

Run it:
```bash
python3 set_baudrate_2m.py
```

**IMPORTANT**: Power cycle the motors after this!

## Step 2: Update Rust Code

Edit `src/main.rs`, find the default baudrate (around line 25):

```rust
// Change from:
#[arg(short, long, default_value_t = 1_000_000)]
baudrate: u32,

// To:
#[arg(short, long, default_value_t = 2_000_000)]
baudrate: u32,
```

## Step 3: Rebuild and Test

```bash
cargo build --release
./target/release/microduck_runtime --model your_model.onnx
```

## Reverting Back to 1M

If 2M causes issues, revert by setting baudrate value to 3 (= 1M):

```python
# Connect at 2M first
controller = Xl330PyController("/dev/ttyUSB0", 2_000_000, 0.1)
# Then change back to 1M
controller.write_raw_data(motor_id, 8, [3])
```

## Baudrate Values Reference

From XL330 manual (register 8):
- 0 = 9,600 bps
- 1 = 57,600 bps
- 2 = 115,200 bps
- 3 = 1M bps (current)
- **4 = 2M bps** ← use this
- 5 = 3M bps
- 6 = 4M bps

Verified from: https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/
