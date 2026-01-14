# Release Notes

## How to Create a Release

1. Update version in `Cargo.toml`
2. Update CHANGELOG.md with changes
3. Commit changes:
   ```bash
   git add Cargo.toml CHANGELOG.md
   git commit -m "chore: bump version to X.Y.Z"
   ```
4. Create and push tag:
   ```bash
   git tag vX.Y.Z
   git push origin main
   git push origin vX.Y.Z
   ```
5. GitHub Actions will automatically:
   - Build ARM64 binary for Raspberry Pi Zero 2W
   - Create release with binary attached
   - Generate release notes

## Installation Command

Users can install the latest release with:

```bash
curl -sSL https://raw.githubusercontent.com/OWNER/microduck_runtime/main/install.sh | bash
```

Replace `OWNER` with your GitHub username.

## Version History

### v0.1.0 (TBD)
- Initial release
- XL330 motor control with rustypot
- Dummy policy with squatting motion
- IMU interface (dummy implementation)
- Current monitoring for legs
- PID gain tuning via CLI
- 50 Hz control loop
- ONNX Runtime placeholder for RL policies
