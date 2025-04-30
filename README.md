## Personal Trainer Drone

### Abstract
This project develops an autonomous drone trainer that tracks and records a user in real-time. Using computer vision and machine learning, the drone tracks a user while capturing video, enabling hands-free recording for sports, and fitness.

### Methodology
This project will utilizes a Parrot Bebop 2 drone as the hardware platform, using its onboard camera and flight capabilities. The Parrot ARSDK is used to establish communication and control between the drone and a ground station. For real-time user tracking, OpenCV will process the video stream to detect and follow the target using computer vision techniques. YOLOv8 is used to to enhance object detection accuracy, particularly in dynamic environments.


 
These are scripts for using the ROS2 package [ros2_bebop_driver](https://github.com/jeremyfix/ros2_bebop_driver) to allow the Parrot Bebop 2 Drone to follow a person in real time.
