# Projektmodul_Elektronik

Repository for ROS2 Humble Package zur Zustandsüberwachung einer Herdplatte mittels USB Kamera und YOLOv11

## Dependencies
benötigte system dependencies können mit rosdep installiert werden:

```
sudo rosdep init
rosdep update
rosdep install --from-paths path/to/Projektmodul_Elektronik -y --ignore-src
```
## Build
```
colcon build --packages-select zustandserkennung --symlink-install
source install/setup.bash
```

## Start
Zum Start beider Nodes:
```
ros2 launch zustandserkennung boiling_detection_local.launch.py 
```

Zum Start der einzelnen Nodes:
```
ros2 run zustandserkennung camera_publisher
```

```
ros2 run zustandserkennung boiling_detection_node
```
