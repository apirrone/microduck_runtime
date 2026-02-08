# Imitation Policy Changes

This document summarizes the changes applied to match the imitation policy handling from `mjlab_microduck/scripts/infer_policy.py`.

## Summary

The changes ensure that the Rust implementation matches the Python reference implementation for:
1. Phase representation (raw value vs cos/sin)
2. Observation ordering for imitation mode
3. Reading gait period from ONNX metadata

## Changes Made

### 1. Phase Representation (`src/observation.rs`)

**Before:** Phase was encoded as `[cos(phase*2π), sin(phase*2π)]` (2D)
**After:** Phase is now a raw value `[0, 1)` (1D)

This matches the Python implementation in `infer_policy.py` lines 217-222.

### 2. Observation Order (`src/observation.rs`)

**For Imitation Mode:**
- **Before:** `[gyro(3), proj_grav(3), joint_pos(14), joint_vel(14), last_action(14), command(3), phase(2)]` = 53D
- **After:** `[command(3), phase(1), gyro(3), proj_grav(3), joint_pos(14), joint_vel(14), last_action(14)]` = 52D

**For Velocity Mode (no change):**
- Remains: `[gyro(3), proj_grav(3), joint_pos(14), joint_vel(14), last_action(14), command(3)]` = 51D

This matches the Python implementation in `infer_policy.py` lines 230-288.

### 3. ONNX Metadata Reading (`src/policy.rs`)

Added ability to read `gait_period` from ONNX model custom metadata:

```rust
fn read_gait_period(session: &Session) -> Option<f64>
```

This matches the Python implementation in `infer_policy.py` lines 62-71.

### 4. Gait Period Priority System (`src/main.rs`)

Implemented priority system for gait period:
1. **Priority 1:** Use ONNX metadata if available
2. **Priority 2:** Fall back to command-line argument

This matches the Python implementation in `infer_policy.py` lines 133-141.

## Backward Compatibility

✅ **Non-imitation policies continue to work unchanged:**
- Observation size remains 51D for velocity tasks
- No phase observation is added
- Command comes last in observation vector

✅ **All existing tests pass**

## Testing

Run the test suite to verify:
```bash
cargo test
```

All 9 unit tests pass, including new tests for the updated phase representation.

## Key Differences from Python

- Python uses `pickle` files for reference motion data (not implemented in Rust yet)
- Python has multiple ONNX policies (walking/standing) - Rust implementation supports this
- Python has action delay buffer - not yet implemented in Rust

## Related Files

- `src/observation.rs` - Observation construction logic
- `src/policy.rs` - ONNX metadata reading
- `src/main.rs` - Gait period priority system
- `~/MISC/mjlab_microduck/scripts/infer_policy.py` - Reference Python implementation
