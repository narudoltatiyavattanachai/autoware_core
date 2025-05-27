# Autoware Core QCS6490 Platform Guide

This guide provides specific information for deploying Autoware Core on the Qualcomm QCS6490 platform, including hardware dependencies, setup instructions, and special considerations.

## Hardware Specifications

The Qualcomm QCS6490 is an advanced SoC (System on Chip) with the following specifications:
- **Architecture**: ARM64 (AArch64)
- **CPU**: Qualcomm Kryo 670, Octa-core (up to 2.7 GHz)
- **GPU**: Qualcomm Adreno 643L
- **DSP**: Qualcomm Hexagon 770
- **NPU**: Qualcomm AI Engine (6th generation)
- **Memory**: Support for LPDDR5 RAM
- **Camera ISP**: Up to three 28MP cameras or one 64MP camera
- **Connectivity**: Wi-Fi 6E, Bluetooth 5.2, 5G

## ROS 2 Humble Setup for QCS6490

### Prerequisites

1. Install Ubuntu 22.04 LTS (Jammy Jellyfish) 64-bit for ARM64
2. Install required packages:
```bash
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install -y curl gnupg lsb-release build-essential cmake git python3-pip
```

3. Set up ROS 2 Humble repositories:
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

### Installing ROS 2 Humble

```bash
sudo apt install -y ros-humble-ros-base
```

For development, additional tools are recommended:
```bash
sudo apt install -y ros-dev-tools
sudo apt install -y ros-humble-ros2bag ros-humble-rosbag2-storage-default-plugins
```

### Environment Setup

1. Set up the ROS 2 environment:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

2. Install colcon build tools:
```bash
pip install -U colcon-common-extensions
```

## Building Autoware Core for QCS6490

### Clone Repository

```bash
mkdir -p ~/autoware_ws/src
cd ~/autoware_ws/src
git clone https://github.com/narudoltatiyavattanachai/autoware_core.git
```

### Install Dependencies

```bash
cd ~/autoware_ws
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro humble
```

### Platform-Specific Dependencies

Install QCS6490-specific packages:
```bash
# Qualcomm GPU/DSP SDKs
sudo apt install -y libqti-qcms # QCS6490 Compute libqti-qcms package
sudo apt install -y qcs6490-audiorouter # For audio routing on QCS6490
sudo apt install -y qcs6490-camera # Camera libraries for QCS6490
sudo apt install -y qcs6490-display # Display libraries for QCS6490
```

### Build Autoware Core

```bash
cd ~/autoware_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Hardware Acceleration Setup

### Adreno GPU Acceleration

1. Install Adreno GPU drivers:
```bash
sudo apt install -y qcs6490-gpu-driver
```

2. Set up OpenCL environment variables:
```bash
echo "export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/qcs6490/opencl:$LD_LIBRARY_PATH" >> ~/.bashrc
source ~/.bashrc
```

### Hexagon DSP Acceleration

1. Install Hexagon DSP SDK:
```bash
sudo apt install -y qcs6490-hexagon-sdk
```

2. Set up environment variables:
```bash
echo "export ADSP_LIBRARY_PATH=/opt/qcs6490/adsp" >> ~/.bashrc
source ~/.bashrc
```

### NPU AI Engine Acceleration

1. Install Qualcomm Neural Processing SDK:
```bash
sudo apt install -y qcs6490-ai-engine-sdk
```

2. Configure paths:
```bash
echo "export QNN_SDK_ROOT=/opt/qcs6490/qnn" >> ~/.bashrc
source ~/.bashrc
```

## Launch Configuration

Launch files can be configured to take advantage of QCS6490 hardware acceleration:

```bash
ros2 launch autoware_launch autoware.launch.xml sensor_model:=qcs6490 vehicle_model:=lexus_accel_type:=gpu
```

Use the following parameters for different acceleration types:
- `accel_type:=gpu` - Use Adreno GPU acceleration
- `accel_type:=dsp` - Use Hexagon DSP acceleration
- `accel_type:=npu` - Use NPU AI Engine acceleration

## Performance Considerations

1. **CPU Frequency Governor**: Set to performance mode for better throughput:
```bash
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

2. **Thermal Management**: Monitor system temperature during intensive workloads:
```bash
watch -n 1 cat /sys/class/thermal/thermal_zone*/temp
```

3. **Power Management**: Connect to stable power source for continuous operation.

4. **Memory Management**: Configure swap space to handle memory-intensive operations:
```bash
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

## Known Issues and Limitations

1. **Perception Module Limitations**: Some perception modules may require specific optimizations for QCS6490.
2. **GPU Memory**: Limited compared to desktop GPUs, requiring careful management of GPU workloads.
3. **Sensor Compatibility**: Some sensors may require specific drivers for QCS6490 platform.
4. **Thermal Throttling**: Intensive computation may trigger thermal throttling, affecting performance.

## Additional Resources

- [Qualcomm QCS6490 Developer Guide](https://developer.qualcomm.com/qcs6490)
- [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/main/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/index.html)