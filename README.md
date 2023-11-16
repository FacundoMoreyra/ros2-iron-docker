# ROS2 Iron Docker

## Features
- Gazebo
- Rviz2
- Python-Websocket to ROS2 interface

## Prerequisites
- docker
- docker-compose
- VSCode
- Dev Containers VSCode extension
- Postman (Optional)

## Preparation
Inside vscode, with Dev Containers installed press ctrl+shift+P and type 'rebuild and reopen in container'.
Wait until docker build the container and then in a new terminal run:
```bash
cd main
```
```bash
pip3 install -r requirements.txt
```

## Examples

### Python to Gazebo ROS Topic Example
Inside a terminal run:
```bash
gazebo --verbose /opt/ros/iron/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world
```
In another terminal inside /main, to make the robot move run:
```bash
python3 start.py
```
And to stop it run:
```bash
python3 stop.py
```

### Python-WebSocket to ROS2 interface
In a terminal inside /main, run:
```bash
python3 websocket-ros2-interface.py
```
It should print "Server started at ws://{local_ip}:{port}".
From a WebSocket Client connect to ws://{local_ip}:{port}/{client_name} and send a json with this format:
```json
{
    "acceleration": {
        "x": 1.0, 
        "y": 2.0, 
        "z": 3.0
    }, 
    "angular_velocity": {
        "x": 0.1, 
        "y": 0.2, 
        "z": 0.3
    }
}
```
Finally you can check if this worked from ROS:
```bash
ros2 topic echo imu_data
```