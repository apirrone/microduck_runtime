FROM ghcr.io/cross-rs/aarch64-unknown-linux-gnu:0.2.5

# Pre-install ARM64 libudev so the release build doesn't need apt-get at compile time.
# The base image already restricts /etc/apt/sources.list to amd64; we just add
# a separate sources file pointing at ports.ubuntu.com for arm64 packages.
RUN set -eux \
 && dpkg --add-architecture arm64 \
 && . /etc/os-release \
 && printf 'deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ %s main restricted\ndeb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ %s-updates main restricted\n' \
      "${UBUNTU_CODENAME:-$VERSION_CODENAME}" "${UBUNTU_CODENAME:-$VERSION_CODENAME}" \
      > /etc/apt/sources.list.d/arm64-ports.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends libudev-dev:arm64 \
 && SO=$(find /usr/lib/aarch64-linux-gnu /lib/aarch64-linux-gnu -name 'libudev.so*' | head -1) \
 && dir=$(dirname "$SO") \
 && { [ -f "$dir/libudev.so" ] || ln -sf "$SO" "$dir/libudev.so"; } \
 && mkdir -p /usr/lib/aarch64-linux-gnu/pkgconfig \
 && printf 'prefix=/usr\nexec_prefix=${prefix}\nlibdir=${exec_prefix}/lib/aarch64-linux-gnu\nincludedir=${prefix}/include\n\nName: libudev\nDescription: Library to access udev device information\nVersion: 1.0\nLibs: -L${libdir} -ludev\nCflags: -I${includedir}\n' \
      > /usr/lib/aarch64-linux-gnu/pkgconfig/libudev.pc \
 && rm -rf /var/lib/apt/lists/*
