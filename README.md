## ROS_WHISPER_PKG

==============
### Example usage:
```
mkdir -p ~/whisper_ws/src
cd ~/whisper_ws/src
git clone git@github.com:CharlesXu1124/ros_whisper_pkg.git
cd ~/whisper_ws
colcon build
source install/setup.bash
ros2 launch ros_whisper_pkg whisper_server.launch.py
```
### Open another terminal
```
cd ~/whisper_ws/src
source install/setup.bash
ros2 service call /whisper_server whisper_interfaces/srv/WhisperResponse "{record_time: '5'}"
```

### Speak something near your microphone and it will return the transcribed results

