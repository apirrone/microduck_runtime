FROM ghcr.io/cross-rs/aarch64-unknown-linux-gnu:0.2.5

# Pre-install ARM64 libudev so the release build doesn't need apt-get at compile time.
# Detects the Ubuntu codename at build time to pick the right ports mirror.
RUN dpkg --add-architecture arm64 \
 && . /etc/os-release \
 && sed -i 's|^deb |deb [arch=amd64] |' /etc/apt/sources.list \
 && printf "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ ${UBUNTU_CODENAME} main restricted universe\ndeb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ ${UBUNTU_CODENAME}-updates main restricted universe\n" >> /etc/apt/sources.list \
 && apt-get update -qq \
 && apt-get install -y --no-install-recommends libudev-dev:arm64 libstdc++6:arm64 \
 && SO=$(find /usr/lib/aarch64-linux-gnu /lib/aarch64-linux-gnu -name 'libudev.so*' | head -1) \
 && dir=$(dirname "$SO") \
 && ([ -f "$dir/libudev.so" ] || ln -sf "$SO" "$dir/libudev.so") \
 && mkdir -p /usr/lib/aarch64-linux-gnu/pkgconfig \
 && printf 'prefix=/usr\nexec_prefix=${prefix}\nlibdir=${exec_prefix}/lib/aarch64-linux-gnu\nincludedir=${prefix}/include\n\nName: libudev\nDescription: Library to access udev device information\nVersion: 1.0\nLibs: -L${libdir} -ludev\nCflags: -I${includedir}\n' \
      > /usr/lib/aarch64-linux-gnu/pkgconfig/libudev.pc \
 && rm -rf /var/lib/apt/lists/*
