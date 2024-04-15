
Launching the Docker container (run the command from inside microros_outputs folder)
```
 ./docker/1_dockerscript.sh 
```
Upload Arduino subscriber Script

Run microros

```
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```






Publishing a float:

```
ros2 topic pub /sammy std_msgs/msg/Float32 data:\ 0.0\
```

modified subscriber: my code
joy subscriber: code that currently works, which turns portenta lights on
micro-ros_light test: code to draw inspiration from as it does cool loading animations with LEDs.

red :live
green: ground
black: data
