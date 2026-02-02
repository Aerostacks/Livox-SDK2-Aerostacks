# Livox-SDK2-Aerostacks

Aerostacks-customized fork of Livox SDK2 for terrain mapping and snow depth measurement missions.

## Overview

This is a customized version of the Livox SDK2 specifically configured for Aerostacks field data collection operations. It provides the LiDAR interface for the Synapse data collection system, enabling high-resolution point cloud capture for terrain mapping and snow depth analysis.

## Aerostacks Customizations

This fork includes Aerostacks-specific modifications:

- **Integration with Synapse** - Configured for seamless integration with the Aerostacks data collection backend
- **Custom configuration** - Pre-configured settings optimized for field operations (`config.json`)
- **Deployment support** - Aero CLI configuration for automated deployment (`aero.yaml`)
- **CMake presets** - Build configurations for Aerostacks hardware (`CMakePresets.json`)

## Supported Hardware

- **Livox HAP** (TX/T1)
- **Livox Mid-360**

These sensors are used in Aerostacks field systems for:
- Terrain mapping
- Snow depth measurement
- Vegetation analysis
- High-resolution 3D scanning

## Quick Start for Aerostacks

### Prerequisites

- Ubuntu 18.04 or above
- CMake 3.0+
- gcc 4.8.1+

### Installation

```bash
# Clone the repository
git clone https://github.com/Aerostacks/Livox-SDK2-Aerostacks.git
cd Livox-SDK2-Aerostacks

# Build and install
mkdir build && cd build
cmake .. && make -j
sudo make install
```

### Configuration

The Aerostacks configuration is defined in `config.json`:

```json
{
  "lidar_configs": [
    {
      "ip": "192.168.1.1",
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "extrinsic_parameter": {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
      }
    }
  ]
}
```

Modify this file to match your LiDAR network configuration and mounting parameters.

## Integration with Synapse

The Synapse data collection system uses this SDK to:

1. Connect to Livox LiDAR sensors
2. Configure scan patterns and data formats
3. Receive real-time point cloud data
4. Save point clouds to mission directories
5. Synchronize with other sensors (Radar, GPS, MSI)

### Usage in Synapse

```python
from lidar import getLidarRunner

# Initialize LiDAR controller
lidar_runner = getLidarRunner(mission_dir)

# Start data collection
lidar_runner.start()

# Stop data collection
lidar_runner.stop()
```

## Deployment with Aero CLI

Deploy to field systems using Aero CLI:

```bash
aero deploy
```

Configuration is defined in `aero.yaml`.

## Project Structure

```
Livox-SDK2-Aerostacks/
├── sdk_core/              # Core SDK implementation
├── include/               # Public API headers
│   └── livox_lidar_api.h # Main API interface
├── samples/               # Example programs
│   ├── livox_lidar_quick_start/
│   └── livox_lidar_imu_data_sample/
├── 3rdparty/             # Third-party dependencies
├── config.json           # Aerostacks LiDAR configuration
├── aero.yaml            # Aero CLI deployment config
├── CMakeLists.txt       # Build configuration
└── CMakePresets.json    # Aerostacks build presets
```

## Building Samples

### Quick Start Sample

```bash
cd build
./livox_lidar_quick_start ../config.json
```

This sample demonstrates:
- LiDAR connection and initialization
- Point cloud data reception
- Basic data processing

### IMU Data Sample

```bash
cd build
./livox_lidar_imu_data_sample ../config.json
```

This sample shows how to receive IMU data from the LiDAR sensor.

## API Reference

The main API is defined in `include/livox_lidar_api.h`. Key functions:

- `LivoxLidarSdkInit()` - Initialize the SDK
- `SetLivoxLidarPointCloudCallBack()` - Register point cloud callback
- `SetLivoxLidarImuDataCallback()` - Register IMU data callback
- `LivoxLidarSdkStart()` - Start data reception
- `LivoxLidarSdkUninit()` - Cleanup and shutdown

For complete API documentation, see the [original Livox SDK2 documentation](https://github.com/Livox-SDK/Livox-SDK2).

## Communication Protocol

This SDK implements the Livox SDK2 Communication Protocol:

**HAP (TX/T1):**
- [HAP Protocol (English)](https://github.com/Livox-SDK/Livox-SDK2/wiki/Livox-SDK-Communication-Protocol-HAP(English))
- [HAP Protocol (中文)](https://github.com/Livox-SDK/Livox-SDK2/wiki/Livox-SDK-Communication-Protocol-HAP)

**Mid-360:**
- [Mid-360 Protocol (English)](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/mid360.html)
- [Mid-360 Protocol (中文)](https://livox-wiki-cn.readthedocs.io/zh_CN/latest/tutorials/new_product/mid360/mid360.html)

## Network Configuration

LiDAR sensors must be accessible on the network:

1. Configure LiDAR IP address (default: 192.168.1.1)
2. Ensure host machine is on same subnet
3. Update `config.json` with correct IP
4. Test connectivity: `ping 192.168.1.1`

## Troubleshooting

### Cannot connect to LiDAR

- Check network connectivity: `ping <lidar-ip>`
- Verify IP address in `config.json`
- Ensure no firewall blocking UDP ports
- Check LiDAR power and initialization

### No point cloud data

- Verify callback registration
- Check LiDAR scan pattern configuration
- Ensure SDK is started: `LivoxLidarSdkStart()`
- Review logs for error messages

### Build errors

- Ensure CMake 3.0+ is installed
- Check compiler supports C++11
- Verify all dependencies are installed
- Try clean build: `rm -rf build && mkdir build`

## Upstream Repository

This is a fork of the official Livox SDK2:
- **Upstream:** https://github.com/Livox-SDK/Livox-SDK2
- **License:** See LICENSE.txt

For general SDK questions, refer to the upstream repository and documentation.

## Related Aerostacks Projects

- [Synapse](https://github.com/Aerostacks/synapse) - Data collection backend using this SDK
- [Aerostacks](https://github.com/Aerostacks/Aerostacks) - Point cloud processing pipeline
- [infra-config](https://github.com/Aerostacks/infra-config) - Deployment configuration

## Contributing

For Aerostacks-specific changes:
1. Fork this repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

For general SDK improvements, contribute to the [upstream Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2).

## License

See LICENSE.txt for details. This project maintains the same license as the upstream Livox SDK2.

## Support

- **Aerostacks issues:** https://github.com/Aerostacks/Livox-SDK2-Aerostacks/issues
- **Livox SDK issues:** https://github.com/Livox-SDK/Livox-SDK2/issues
- **Livox documentation:** https://www.livoxtech.com/
