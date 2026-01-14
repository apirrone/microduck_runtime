# GitHub Setup Guide

This guide explains how to set up the repository on GitHub for automatic releases.

## Prerequisites

- GitHub account
- Repository created on GitHub
- Git configured locally

## Step 1: Update Repository References

Replace `OWNER` with your GitHub username in these files:

### 1. `install.sh`
```bash
REPO="${MICRODUCK_REPO:-OWNER/microduck_runtime}"
```
Change `OWNER` to your username (e.g., `apirrone/microduck_runtime`)

### 2. `README.md`
Update all URLs that contain `OWNER`:
- Installation commands
- Release links
- Clone URLs

### 3. `.github/workflows/release.yml`
The workflow uses `${{ github.repository }}` so it will automatically use the correct repo.

## Step 2: Push to GitHub

```bash
# Add remote if not already added
git remote add origin https://github.com/OWNER/microduck_runtime.git

# Push code
git add .
git commit -m "Initial commit with CI/CD setup"
git push -u origin main
```

## Step 3: Create First Release

```bash
# Create and push a tag
git tag v0.1.0
git push origin v0.1.0
```

This will trigger the GitHub Actions workflow which will:
1. Cross-compile for ARM64 (Raspberry Pi Zero 2W)
2. Create a release
3. Attach the binary to the release

## Step 4: Verify Release

1. Go to `https://github.com/OWNER/microduck_runtime/releases`
2. You should see release `v0.1.0` with:
   - Release notes
   - Attached binary: `microduck_runtime-aarch64-linux.tar.gz`

## Step 5: Test Installation

On your Raspberry Pi Zero 2W:

```bash
curl -sSL https://raw.githubusercontent.com/OWNER/microduck_runtime/main/install.sh | bash
```

## Troubleshooting

### Workflow Fails

Check the Actions tab on GitHub:
- `https://github.com/OWNER/microduck_runtime/actions`

Common issues:
- **Permissions**: Make sure workflow has write permissions (Settings > Actions > General > Workflow permissions)
- **Cross-compilation**: The workflow uses `cross` which requires Docker
- **OpenSSL errors**: We use `openssl = { version = "0.10", features = ["vendored"] }` in Cargo.toml to compile OpenSSL from source during cross-compilation. This avoids missing OpenSSL development libraries in the cross-compilation environment.

### Binary Not in Release

- Check if the workflow completed successfully
- Verify the `files:` section in `.github/workflows/release.yml` matches the actual file path

### Installation Script Fails

- Verify the release exists and contains the binary
- Check that the REPO variable in `install.sh` is correct
- Ensure GitHub release is public (not draft)

## Alternative: Manual Release

If GitHub Actions doesn't work, you can create releases manually:

```bash
# On a Linux machine with cross installed
cargo install cross
cross build --release --target aarch64-unknown-linux-gnu

# Create tarball
tar czf microduck_runtime-aarch64-linux.tar.gz -C target/aarch64-unknown-linux-gnu/release microduck_runtime

# Upload to GitHub releases manually via web interface
```

## Environment Variables

The install script supports environment variables:

```bash
# Use a custom repository
MICRODUCK_REPO="custom-user/custom-repo" curl -sSL https://raw.githubusercontent.com/.../install.sh | bash
```

## GitHub Actions Configuration

The workflow is in `.github/workflows/release.yml` and:
- Triggers on tags matching `v*`
- Uses `cross` for cross-compilation
- Requires `GITHUB_TOKEN` (automatically provided by GitHub)
- Needs write permissions to create releases

### Permissions

If releases don't get created, check repository settings:
1. Go to Settings > Actions > General
2. Under "Workflow permissions", select "Read and write permissions"
3. Save

## Next Steps

Once everything is working:
1. Update version in `Cargo.toml` for new releases
2. Tag and push: `git tag vX.Y.Z && git push origin vX.Y.Z`
3. GitHub Actions automatically builds and releases
4. Users install with the curl command

## Maintenance

For subsequent releases:
1. Make changes and commit
2. Update version in `Cargo.toml`
3. Create new tag: `git tag vX.Y.Z`
4. Push tag: `git push origin vX.Y.Z`
5. GitHub Actions handles the rest!
