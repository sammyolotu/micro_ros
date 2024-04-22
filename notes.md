
# Tests

## Test Connection between microros and the bigger Diegetic system

### 1. Microros Subscriber

1. Upload Arduino subscriber Script (something.uf2) from the Arduino IDE (connect to Vin and GPIO4 pins)
2. Launch the ROS2 environment on Docker. For this, go to the micro_ros workspace and run:

```
 ./docker/1_dockerscript.sh 
```

3. Source the installation and run microros to create the Arduino subscriber (connect ot the Arduino)

```
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

### 2. Joy-message ROS2 publisher

1. Launch a Docker environment with the joy package. Go to the jaco-docker workspace next to the microros one. Many would work. Go to the workspace and run:

```
 ./docker/1_dockerscript.sh 
```

2. From this Docker environment, we have two options:

2a. Emulate the joystick input:

```
ros2 topic pub /joy sensor_msgs/msg/Joy " (press tab here)
```

2b. Run and connect to a real joystick

```
ros2 run joy joy_node 
```

# Other

To re-upload
1. compile arduino code
2. Double press arduino reset button
3. type control-c in terminal
4. disconnect and re-connect arduino
5. re-run ros2 micro-ros agent in terminal


Publishing a float:

```
ros2 topic pub /sammy std_msgs/msg/Float32 data:\ 0.0\
```

modified subscriber: my code
joy subscriber: code that currently works, which turns portenta lights on
micro-ros_light test: code to draw inspiration from as it does cool loading animations with LEDs.

GREEN :live
black: ground
yellow: data
