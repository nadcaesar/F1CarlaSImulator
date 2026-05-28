# 🏎️ F1 CARLA Simulator

A Formula One vehicle simulation project built in C++ using the CARLA open-source autonomous driving simulator. This project models F1 car physics including tire dynamics, aerodynamics, and race telemetry — designed as a portfolio project demonstrating systems programming, real-time simulation, and applied machine learning.

**Author:** Nicholas Caesar  
**Degree:** B.S. Computer Science, Howard University (December 2026)  
**Stack:** C++, Python, CARLA 0.9.16, Ubuntu 22.04, ROS

---

## 📋 Table of Contents

- [Project Overview](#project-overview)
- [Project Goals](#project-goals)
- [Architecture](#architecture)
- [Environment Setup](#environment-setup)
- [Installation](#installation)
- [Usage](#usage)
- [Results](#results)
- [Development Journal](#development-journal)
- [Roadmap](#roadmap)
- [Lessons Learned](#lessons-learned)

---

## Project Overview

This simulator models a Formula One car operating on a race circuit, capturing real-time telemetry data including:

- Tire wear and degradation models
- Aerodynamic downforce and drag calculations
- Lap time analysis and sector breakdowns
- Fuel consumption modeling
- Pit strategy simulation

The project is built on top of CARLA — the same open-source simulator used by autonomous vehicle companies like Waymo and Aurora — with custom C++ clients that interface with CARLA's API to control vehicle behavior and extract simulation data.

---

## Project Goals

### Short Term (Summer 2026)

- [ ] Get CARLA running with a basic vehicle model
- [ ] Write C++ client that connects to CARLA and controls a vehicle
- [ ] Log basic telemetry (speed, position, tire data) to CSV
- [ ] Implement basic F1 tire degradation model
- [ ] Push clean, documented results to GitHub

### Medium Term (Autumn 2026)

- [ ] Implement aerodynamics model (downforce, drag, DRS)
- [ ] Build lap time analyzer
- [ ] Add pit strategy logic layer
- [ ] Create visualization of telemetry data
- [ ] Polish GitHub repo for portfolio use

### Long Term

- [ ] Integrate machine learning for optimal race strategy
- [ ] Compare simulated telemetry against real F1 data
- [ ] Build web dashboard for visualizing simulation results

---

## Architecture

markdown

```
f1-carla-simulator/
├── src/
│   ├── main.cpp                  # Entry point, CARLA client init
│   ├── vehicle/
│   │   ├── vehicle.h             # Vehicle class definition
│   │   └── vehicle.cpp           # Vehicle physics implementation
│   ├── track/
│   │   ├── track.h               # Track class definition
│   │   └── track.cpp             # Track geometry and sector logic
│   ├── telemetry/
│   │   ├── telemetry.h           # Telemetry logger definition
│   │   └── telemetry.cpp         # Data capture and CSV export
│   └── utils/                    # Shared utility functions
├── results/
│   ├── telemetry/                # CSV telemetry output files
│   ├── logs/                     # Simulation run logs
│   └── replays/                  # Saved simulation replays
├── docs/
│   ├── research/                 # Research notes and references
│   └── diagrams/                 # Architecture diagrams
├── tests/                        # Unit tests
├── config/
│   └── simulation_config.json    # Simulation parameters
└── README.md
```

---

## Environment Setup

### Host Machine Specs

| Component   | Spec                                  |
| ----------- | ------------------------------------- |
| **CPU**     | Intel Core i7-13700F (13th Gen)       |
| **RAM**     | 32GB                                  |
| **Storage** | 2TB                                   |
| **GPU**     | Dedicated (NVIDIA)                    |
| **OS**      | Windows 11 / Ubuntu 22.04 (Dual Boot) |

### Virtual Machine Specs (Development Phase)

| Setting        | Value                      |
| -------------- | -------------------------- |
| **Hypervisor** | Oracle VirtualBox 7.0+     |
| **OS**         | Ubuntu 22.04 LTS           |
| **RAM**        | 16GB allocated             |
| **CPU Cores**  | 4-6 cores                  |
| **Storage**    | 150GB virtual disk         |
| **Graphics**   | VMSVGA, 128MB video memory |

### Software Stack

- **CARLA** 0.9.16 — Autonomous driving simulator
- **Unreal Engine** 4.26 — CARLA's rendering backend
- **VS Code** — Primary IDE
- **GCC/G++** — C++ compiler
- **CMake** — Build system
- **Python 3.10** — Scripting and data analysis
- **Git** — Version control

---

## Installation

### Prerequisites

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install build tools
sudo apt install -y git build-essential cmake python3-pip

# Install VS Code
sudo snap install code --classic

# Install CARLA Python client
pip3 install carla==0.9.16
```

### Clone Repository

```bash
cd ~/Documents
git clone https://github.com/YOUR-USERNAME/f1-carla-simulator.git
cd f1-carla-simulator
```

### Install CARLA

```bash
# Download CARLA 0.9.16
cd ~/Downloads
wget -L https://github.com/carla-simulator/carla/releases/download/0.9.16/CARLA_0.9.16.tar.gz

# Extract
mkdir -p ~/Documents/CARLA
tar -xvf CARLA_0.9.16.tar.gz -C ~/Documents/CARLA

# Make executable
cd ~/Documents/CARLA
chmod +x CarlaUE4.sh
```

### Launch CARLA

```bash
# Standard launch
./CarlaUE4.sh

# Inside VirtualBox VM (OpenGL mode)
./CarlaUE4.sh -opengl

# Headless mode (no rendering, data only)
./CarlaUE4.sh -RenderOffScreen -nosound
```

### Build Project

```bash
cd ~/Documents/f1-carla-simulator
mkdir build && cd build
cmake ..
make
```

---

## Usage

```bash
# Start CARLA server first
cd ~/Documents/CARLA
./CarlaUE4.sh

# Then in a new terminal, run the simulator
cd ~/Documents/f1-carla-simulator/build
./f1_simulator --track monza --laps 10 --output ../results/telemetry/
```

---

## Results

Simulation telemetry outputs are stored in `results/telemetry/` as CSV files with the following columns:

| Column           | Description                |
| ---------------- | -------------------------- |
| `timestamp`      | Simulation time in seconds |
| `speed_kmh`      | Vehicle speed in km/h      |
| `throttle`       | Throttle input (0.0 - 1.0) |
| `brake`          | Brake input (0.0 - 1.0)    |
| `steering`       | Steering angle             |
| `tire_fl_wear`   | Front left tire wear %     |
| `tire_fr_wear`   | Front right tire wear %    |
| `tire_rl_wear`   | Rear left tire wear %      |
| `tire_rr_wear`   | Rear right tire wear %     |
| `fuel_remaining` | Fuel remaining in kg       |
| `lap_number`     | Current lap                |
| `sector`         | Current track sector (1-3) |

---

## Development Journal

This section documents the real setup process including every obstacle encountered and how it was resolved. This is kept intentionally honest because debugging is real engineering work.

---

### Phase 1 — Initial Environment Setup

**Goal:** Get Ubuntu VM running with CARLA installed.

**Environment:**

- Host: Windows 11, i7-13700F, 32GB RAM
- Hypervisor: VirtualBox 7.0
- Target: Ubuntu 22.04 + CARLA 0.9.16

---

#### Issue 1: Ubuntu Version Incompatibility

**Problem:** Initially installed Ubuntu 24.04. CARLA 0.9.15/0.9.16 was built against Ubuntu 22.04 libraries. Running CARLA on 24.04 produced an `Illegal instruction (core dumped)` error immediately on launch.

**Root Cause:** Ubuntu 24.04 ships with updated versions of core system libraries (libc, OpenGL/Vulkan) that CARLA was not compiled against.

**Resolution:** Deleted VM, downloaded Ubuntu 22.04.3 LTS ISO specifically, rebuilt VM from scratch.

**Lesson:** Always check the simulator's officially supported OS version before installing. CARLA's documentation explicitly states Ubuntu 22.04.

---

#### Issue 2: Bidirectional Clipboard Not Working

**Problem:** Copy/paste between Windows host and Ubuntu VM didn't work despite setting Shared Clipboard to Bidirectional in VirtualBox settings.

**Root Cause:** VirtualBox Guest Additions were not properly installed inside the VM. The `virtualbox-guest-dkms` package name changed in Ubuntu 22.04 repositories.

**Resolution:**

```bash
# Remove old broken packages
sudo apt purge virtualbox-guest-dkms virtualbox-guest-x11 virtualbox-guest-utils -y
sudo apt autoremove -y

# Install dependencies
sudo apt update
sudo apt install -y build-essential dkms linux-headers-$(uname -r)
sudo apt install -y virtualbox-guest-utils virtualbox-guest-x11

# Install from VirtualBox CD
sudo mkdir -p /mnt/cdrom
sudo mount /dev/cdrom /mnt/cdrom
cd /mnt/cdrom
sudo ./VBoxLinuxAdditions.run
sudo reboot
```

**Lesson:** Always install Guest Additions from the official VirtualBox CD image rather than relying solely on apt packages, as package names change between Ubuntu versions.

---

#### Issue 3: AVX2 CPU Instructions Not Passing Through VM

**Problem:** CARLA crashed with `Illegal instruction (core dumped)` even on Ubuntu 22.04. The command `grep avx2 /proc/cpuinfo` returned no output inside the VM, meaning AVX2 instructions were not visible despite the host CPU (i7-13700F) fully supporting them.

**Root Cause:** Windows Virtualization Based Security (VBS) was running and intercepting the CPU virtualization layer before VirtualBox could access it. This prevented VirtualBox from passing advanced CPU instruction sets (AVX/AVX2) through to the VM.

**Diagnosis Process:**

```bash
# On Windows - confirmed VBS was active
systeminfo | findstr /i "virtualization"
# Output: Virtualization-based Security: Status: Running
```

**Failed Attempts:**

1. `bcdedit /set hypervisorlaunchtype off` — VBS persisted
2. `bcdedit /set vsmlaunchtype off` — VBS persisted
3. Registry edit `EnableVirtualizationBasedSecurity = 0` — VBS persisted
4. BIOS check — Intel Virtualization Technology and AVX/AVX2 were already Enabled in BIOS

**Root Cause (Deeper):** Norton 360 antivirus was re-enabling VBS through its Tamper Protection feature, overriding the bcdedit and registry changes on every boot.

**Resolution:**

1. Opened Windows Security → Device Security → Core Isolation Details
2. Disabled **Memory Integrity**
3. Full power cycle (not restart)
4. Confirmed VBS disabled:

```bash
systeminfo | findstr /i "virtualization"
# Output: Virtualization-based Security: Status: Not Enabled
```

5. Re-ran VBoxManage AVX2 commands:

```bash
VBoxManage setextradata "F1-Simulator-Dev" VBoxInternal/CPUM/IsaExts/AVX 1
VBoxManage setextradata "F1-Simulator-Dev" VBoxInternal/CPUM/IsaExts/AVX2 1
```

6. Confirmed AVX2 visible in VM:

```bash
grep avx2 /proc/cpuinfo | head -1
# Output: flags : ... avx2 ... (success)
```

**Lesson:** Security software like Norton actively fights VBS changes. Always check Memory Integrity under Core Isolation in Windows Security when VirtualBox can't access CPU features. The fix is in Windows Security, not the registry or bcdedit.

---

#### Issue 4: CARLA Segmentation Fault on Launch

**Problem:** After resolving AVX2, CARLA launched but crashed with:

markdown`
GameThread timed out waiting for RenderThread after 60.00 secs
Segmentation fault (core dumped)`

**Root Cause:** VirtualBox's virtual GPU cannot handle Unreal Engine 4's Vulkan rendering pipeline. Even with AVX2 resolved, the virtual display adapter lacks the graphics capability CARLA requires for rendering.

**Current Status:** This is a fundamental VirtualBox limitation. CARLA requires direct GPU access for its rendering engine.

**Resolution (In Progress):** Dual booting Ubuntu 22.04 natively on the host machine to give CARLA direct access to the dedicated GPU. This is the proper long-term solution.

---

### Phase 2 — Dual Boot Setup _(In Progress)_

**Goal:** Install Ubuntu 22.04 natively alongside Windows 11 to give CARLA full GPU access.

**Plan:**

- Back up Windows to external drive using Macrium Reflect
- Shrink Windows partition by 80GB using Disk Management
- Install Ubuntu 22.04 into freed space
- Configure GRUB dual boot
- Install CARLA with full GPU support
- Verify CARLA launches with 3D environment

**Status:** Pending external drive acquisition for backup.

---

## Roadmap

### ✅ Completed

- Project repository created and structured
- Ubuntu 22.04 VM configured with AVX2 support
- VBS disabled, Guest Additions working
- Clipboard bidirectional working
- Dev environment: Git, VS Code, G++, CMake installed
- GitHub repo connected

### 🔄 In Progress

- Dual boot Ubuntu setup on home PC
- CARLA confirmed running natively

### 📋 Upcoming

- Write first C++ CARLA client
- Implement vehicle physics model
- Build telemetry logger
- Add tire degradation model
- Add aerodynamics model
- Set up Tailscale remote access from laptop
- Build results visualization

---

## Lessons Learned

1. **Check OS compatibility first** — Always verify the simulator's supported OS before installing anything. One version difference caused hours of debugging.

2. **Security software fights you** — Norton 360's Tamper Protection actively re-enabled Windows VBS overriding manual registry and bcdedit changes. The fix was in Windows Security → Core Isolation, not the command line.

3. **VirtualBox has real GPU limits** — For graphics-intensive applications like CARLA/Unreal Engine, VirtualBox's virtual GPU is not sufficient. Dual boot or WSL2 with GPU passthrough are the proper solutions.

4. **Document everything** — Every error message, every failed command, every fix. This README exists because of that habit and it's already useful reference material.

5. **Guest Additions installation method matters** — Installing from the VirtualBox CD image (`VBoxLinuxAdditions.run`) is more reliable than apt packages alone on Ubuntu 22.04.

---

## References

- [CARLA Documentation](https://carla.readthedocs.io)
- [CARLA GitHub Releases](https://github.com/carla-simulator/carla/releases)
- [ROS Documentation](https://docs.ros.org)
- [VirtualBox Guest Additions Guide](https://www.virtualbox.org/manual/ch04.html)
- [Ubuntu 22.04 LTS Download](https://releases.ubuntu.com/22.04/)

---

## License

MIT License — feel free to use, modify, and distribute.

---

## Tools & Acknowledgements

This project was built with assistance from **Claude (Anthropic)** as an AI pair programming and debugging partner. Claude was used throughout the development process for:

- Architectural planning and project structure decisions
- Debugging environment setup issues (VBS, AVX2, Guest Additions)
- Researching CARLA compatibility requirements
- Writing and reviewing C++ code
- Documenting the development process

All code, decisions, and implementations were reviewed, understood, and executed by the developer. Claude served as a technical sounding board — similar to how a senior engineer or mentor might assist a junior developer working through a complex setup.

AI-assisted development is a core skill in modern software engineering. This project reflects the ability to effectively leverage AI tools while maintaining full ownership of the technical work.

---
