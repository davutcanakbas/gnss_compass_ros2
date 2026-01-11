# GNSS Compass Driver for ROS2

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS2](https://img.shields.io/badge/ROS2-Humble%20|%20Iron%20|%20Jazzy-green)](https://docs.ros.org/)

Professional-grade ROS2 driver for dual-antenna GNSS receivers with RTK support and true heading capability.

> ⚠️ **Disclaimer**: This is an **independent open-source project** and is **NOT officially affiliated with or endorsed by ArduSimple, u-blox, or Unicore**. This driver is developed and maintained by the community for **educational, research, and hobbyist purposes**. Use at your own risk. No warranty is provided. For commercial or safety-critical applications, please consult the official documentation and support channels of your hardware manufacturer.

## ✨ Features

- 🛰️ **Dual-Antenna Heading** - True heading from GNSS (no magnetometer drift)
- 📍 **RTK Positioning** - Centimeter-level accuracy with corrections
- 🌐 **Built-in NTRIP Client** - Connect to free or paid RTK services
- ⚡ **Multi-Protocol** - NMEA, UBX (u-blox), and UniCore support
- 🔄 **Lifecycle Node** - Full ROS2 lifecycle management
- 🔌 **Auto-Reconnect** - Robust serial port handling
- 📊 **Diagnostics** - Comprehensive status monitoring
- 🧩 **Composable** - Zero-copy intra-process communication

## 🔧 Supported Hardware

| Manufacturer | Model | Chip | Status |
|-------------|-------|------|--------|
| **ArduSimple** | simpleRTK3B Compass | Unicore UM982 | ✅ Tested |
| **ArduSimple** | simpleRTK2B Heading | u-blox ZED-F9P | ✅ Supported |
| **u-blox** | ZED-F9P + ZED-F9H | u-blox | ✅ Supported |
| **Unicore** | UM982 | Unicore | ✅ Supported |
| Other | Dual-antenna GNSS | NMEA/UBX/UniCore | Should work |

## 📦 Installation

### Prerequisites

```bash
# ROS2 dependencies
sudo apt install ros-${ROS_DISTRO}-tf2-geometry-msgs

# Serial port access
sudo usermod -a -G dialout $USER
# Log out and back in for group change to take effect
```

### Build from Source

```bash
cd ~/ros2_ws/src
git clone https://github.com/davutcanakbas/gnss_compass_ros2.git gnss_compass_driver
cd ..
colcon build --packages-select gnss_compass_driver
source install/setup.bash
```

## 🚀 Quick Start

### Basic Usage (Without RTK)

```bash
# Launch the driver (standard node)
ros2 launch gnss_compass_driver gnss_compass.launch.py

# Or launch as composable node (zero-copy, better performance)
ros2 launch gnss_compass_driver gnss_compass_component.launch.py

# Check data
ros2 topic echo /gnss/fix
ros2 topic echo /gnss/heading/degrees
```

### Launch File Differences

| Launch File | Type | Use Case |
|-------------|------|----------|
| `gnss_compass.launch.py` | Standard Node | ✅ **Recommended for most users** - Simple, standalone |
| `gnss_compass_component.launch.py` | Composable Node | For high-performance systems with multiple components in same process |

### With RTK Corrections (Centimeter Accuracy)

```bash
# Step 1: Find a nearby free RTK base station
ros2 run gnss_compass_driver find_rtk2go_stations.sh YOUR_LAT YOUR_LON

# Step 2: Launch with NTRIP client
ros2 launch gnss_compass_driver gnss_compass.launch.py &
ros2 run gnss_compass_driver ntrip_client --ros-args \
    -p mountpoint:=YOUR_STATION \
    -p username:=your.email@example.com
```

## 🛰️ RTK Setup

RTK provides **centimeter-level accuracy** using corrections from base stations.

### Finding Free Base Stations

```bash
# Find stations near your location
ros2 run gnss_compass_driver find_rtk2go_stations.sh <lat> <lon>

# Examples
ros2 run gnss_compass_driver find_rtk2go_stations.sh 51.50 -0.12  # London
ros2 run gnss_compass_driver find_rtk2go_stations.sh 40.71 -74.00 # New York
ros2 run gnss_compass_driver find_rtk2go_stations.sh 35.68 139.69 # Tokyo

# List by country
ros2 run gnss_compass_driver find_rtk2go_stations.sh --country USA
ros2 run gnss_compass_driver find_rtk2go_stations.sh --country DEU
```

### RTK Accuracy vs Distance

| Distance to Base | Quality | Accuracy |
|-----------------|---------|----------|
| < 35 km | ✅ Excellent | 1-2 cm |
| 35-50 km | ⚠️ Usable | 2-5 cm |
| > 50 km | ❌ Use Network RTK | - |

### Free NTRIP Services

| Service | Coverage | URL |
|---------|----------|-----|
| **RTK2GO** | Worldwide | [rtk2go.com](http://rtk2go.com) |
| **Centipede** | Europe | [centipede.fr](https://centipede.fr) |

### Paid Network RTK (VRS)

For areas without nearby free stations, Network RTK creates a virtual base at your location:

| Country | Service |
|---------|---------|
| Turkey | [TUSAGA-Aktif](https://tusaga-aktif.gov.tr) |
| Germany | [SAPOS](https://sapos.de) |
| USA | Various state networks |

## 📡 Topics

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `gnss/fix` | `sensor_msgs/NavSatFix` | Position with covariance |
| `gnss/heading/quaternion` | `geometry_msgs/QuaternionStamped` | Heading as quaternion |
| `gnss/heading/degrees` | `std_msgs/Float64` | Heading (0-360°) |
| `gnss/velocity` | `geometry_msgs/TwistStamped` | Velocity (ENU) |
| `gnss/time_reference` | `sensor_msgs/TimeReference` | GPS time |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Status |

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `rtcm` | `std_msgs/UInt8MultiArray` | RTK corrections |

## ⚙️ Parameters

### Driver

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | `/dev/ttyUSB0` | Serial port |
| `baudrate` | `115200` | Baud rate |
| `frame_id` | `gnss_link` | TF frame |
| `use_ubx` | `true` | u-blox UBX protocol |
| `use_nmea` | `true` | NMEA protocol |
| `use_unicore` | `true` | UniCore protocol |
| `antenna_baseline` | `1.0` | Antenna separation (m) |

### NTRIP Client

| Parameter | Default | Description |
|-----------|---------|-------------|
| `host` | `rtk2go.com` | Caster hostname |
| `port` | `2101` | Caster port |
| `mountpoint` | `""` | Base station ID |
| `username` | `""` | Auth (email for RTK2GO) |

## 📊 Status Indicators

### Fix Type

| Status | Description |
|--------|-------------|
| `NO_FIX` | No satellite fix |
| `3D_FIX` | Standard GNSS (~5-15m) |

### RTK Status

| Status | Accuracy | Description |
|--------|----------|-------------|
| `NONE` | 5-15 m | No corrections |
| `FLOAT` | 0.5-1 m | Converging |
| `FIXED` | **1-2 cm** | Full RTK |

## 🏗️ Architecture

```
┌────────────────────────────────────────────────────────────┐
│                  GnssCompassDriverNode                     │
│                   (Lifecycle Node)                         │
├────────────────────────────────────────────────────────────┤
│                                                            │
│  SerialPort ──▶ Parsers ──▶ Publishers                    │
│  (async)       (NMEA/UBX/   (NavSatFix, Heading,          │
│                 UniCore)     Velocity, Diagnostics)        │
│       ▲                                                    │
│       │ RTCM                                               │
│       └─────── /rtcm topic ◀── NtripClientNode            │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

## 🔍 Troubleshooting

### No RTK Fix
1. Check base station distance (< 50km required)
2. Verify NTRIP connection in logs
3. Ensure clear sky view
4. Wait 1-5 minutes for convergence

### No Heading
1. Verify both antennas connected
2. Check antenna baseline configuration
3. Need RTK fix between antennas

### Permission Denied
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

## 📄 License

Apache-2.0 License - See [LICENSE](LICENSE)

## 🤝 Contributing

Contributions welcome! Please open issues or pull requests.

## 👤 Author

**Davut Can Akbas**
- Email: akbasdavutcan@gmail.com
- GitHub: [@davutcanakbas](https://github.com/davutcanakbas)

## 🙏 Acknowledgments

- [RTK2GO](http://rtk2go.com) - Free RTK corrections
- [ArduSimple](https://ardusimple.com) - Hardware
- ROS2 Community

## ⚖️ Disclaimer

This software is provided "as is", without warranty of any kind, express or implied. This is an **independent community project** developed for educational, research, and hobbyist purposes.

**This project is NOT:**
- Officially affiliated with ArduSimple, u-blox, Unicore, or any hardware manufacturer
- Certified for safety-critical, commercial, or production applications
- A replacement for official manufacturer software or support

**Intended use:**
- 🎓 Educational and academic research
- 🔬 Robotics prototyping and experimentation
- 🛠️ Hobbyist and maker projects
- 📚 Learning ROS2 and GNSS technology

For commercial or safety-critical applications, please use manufacturer-provided software and consult their official support channels.
