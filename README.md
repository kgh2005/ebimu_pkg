# ebimu_pkg
**IMU** package

## Development Environment

| Component   | Version          |
|-------------|------------------|
| **OS**      | Ubuntu 22.04     |
| **ROS**     | Humble Hawksbill    |
| **IMU**     | EBIMU    |

## Build

```bash
colcon build --packages-select ebimu_pkg
```

## Run

```bash
ros2 run ebimu_pkg imu_pub
```
