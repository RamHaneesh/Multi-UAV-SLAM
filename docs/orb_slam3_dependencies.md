**[⬅️ Back to README](../README.md)**

## **ORB-SLAM3 Dependencies**

This document covers installation and verification of all dependencies required before building ORB-SLAM3 and its ROS2 wrapper.

**System:** Ubuntu 22.04, ROS2 Humble

---

### **1. OpenCV**

Required version: ≥ 4.2. Ubuntu 22.04 ships with OpenCV 4.5+, and a newer version may already be present.

**Verify (C++ libraries — what ORB-SLAM3 actually uses):**
```bash
pkg-config --modversion opencv4
find /usr/lib /usr/local/lib -name "libopencv_core*" 2>/dev/null
```
Expected: version ≥ 4.2 and `libopencv_core.so` present (confirmed: 4.5.4 on this system).

If not installed:
```bash
sudo apt install -y libopencv-dev
```
> **Note:** ORB-SLAM3 is C++ and links against `libopencv-dev` at compile time. `pip3 install opencv-python` only installs Python bindings — it does not provide the C++ headers and `.so` files the compiler needs. Both can coexist: `pip3` gives Python 4.10.0, `apt` gives C++ 4.5.4, and ORB-SLAM3 uses the latter.

---

### **2. Eigen3**

Required version: ≥ 3.1.

**Verify:**
```bash
pkg-config --modversion eigen3
```
Expected: any version ≥ 3.1 (confirmed: 3.4.0 on this system).

If not installed:
```bash
sudo apt install -y libeigen3-dev
```

---

### **3. Pangolin**

Pangolin is the 3D visualizer used by ORB-SLAM3. It must be built from source. Version 0.9.4 installs as multiple component libraries (`libpango_core`, `libpango_display`, etc.) rather than a single `libpangolin.so` — this is expected.

**Install system dependencies:**
```bash
sudo apt install -y libgl1-mesa-dev libwayland-dev libxkbcommon-dev \
  wayland-protocols libegl1-mesa-dev libc++-dev libglew-dev \
  libeigen3-dev cmake g++ ninja-build
```

**Clone and build:**
```bash
cd ~
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
./scripts/install_prerequisites.sh recommended
cmake -B build -GNinja
cmake --build build -j$(nproc)
sudo cmake --install build
```

**Add to dynamic linker path** (required for runtime — add to `~/.bashrc`):
```bash
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

**Refresh linker cache:**
```bash
sudo ldconfig
```

**Verify:**
```bash
ldconfig -p | grep pango_core
```
Expected output:
```
libpango_core.so.0 (libc6,x86-64) => /usr/local/lib/libpango_core.so.0
libpango_core.so   (libc6,x86-64) => /usr/local/lib/libpango_core.so
```

> **Note:** `pkg-config --modversion pangolin` will return "not found" — this is normal. Pangolin 0.9.4 does not install a `.pc` file. ORB-SLAM3 finds it via CMake's `find_package(Pangolin)` using the config file at `/usr/local/lib/cmake/Pangolin/PangolinConfig.cmake`, which is present and correct.

---

### **Summary**

| Dependency | Version Confirmed | Verification Command |
|---|---|---|
| OpenCV (C++) | 4.5.4 | `pkg-config --modversion opencv4` |
| Eigen3 | 3.4.0 | `pkg-config --modversion eigen3` |
| Pangolin | 0.9.4 | `ldconfig -p \| grep pango_core` |