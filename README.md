## Personal Trainer Drone

These are scripts for using the ROS2 package [ros2_bebop_driver](https://github.com/jeremyfix/ros2_bebop_driver) to allow the Parrot Bebop 2 Drone to follow a person in real time.

### Abstract
This project develops an autonomous drone trainer that tracks and records a user in real-time. Using computer vision and machine learning, the drone tracks a user while capturing video, enabling hands-free recording for sports, and fitness.

### Methodology
This project will utilizes a Parrot Bebop 2 drone as the hardware platform, using its onboard camera and flight capabilities. The Parrot ARSDK is used to establish communication and control between the drone and a ground station. For real-time user tracking, OpenCV will process the video stream to detect and follow the target using computer vision techniques. YOLOv8 is used to enhance object detection accuracy, particularly in dynamic environments.

![Drone](https://platform.theverge.com/wp-content/uploads/sites/2/chorus/uploads/chorus_asset/file/13064385/Parrot-Bebop_2-review-07.0.0.1453474367.jpg?quality=90&strip=all&crop=0,0,100,100)

## How to use

Follow the install for the [ros2_bebop_driver](https://github.com/jeremyfix/ros2_bebop_driver). Then download flight.py, tracking.py, and CMakeLists.txt and place them inside the scripts folder inside the ros2_bebop_driver package. 

Replace the CMakeLists.txt file in the ros2_bebop_driver package with the one provided. 

Then you need to make the flight.py and tracking.py executables :

```
chmod +x scripts/flight.py scripts/tracking.py
```

Now connect to the drone in a terminal :

```
source ros2_ws/install/setup.bash
ros2 launch ros2_bebop_driver bebop_node_launch.xml ip:=BEBOP.IP.ADDRESS
```

Open a new terminal and run :

```
cd /ros2_ws
colcon build
source install/setup.bash
```

Now open two other terminals and run :

```
source install/setup.bash
ros2 run ros2_bebop_driver flight.py
```

```
source install/setup.bash
ros2 run ros2_bebop_driver tracking.py
```

[Here is our website](https://warren.sewanee.edu/darbnjr0/robotic/drone.html)

## Requirements

[ros2_bebop_driver](https://github.com/jeremyfix/ros2_bebop_driver)

[YOLOv8](https://github.com/ultralytics/ultralytics)

[Open CV](https://github.com/ultralytics/ultralytics) 

[ROS2_foxy](https://docs.ros.org/en/foxy/index.html)

[Ubuntu 20.04](https://www.releases.ubuntu.com/focal/)
 
 
 
 
