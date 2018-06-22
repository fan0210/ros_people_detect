# ros_people_detect

A ros warpper for jetson-inference people detect

## Building ros_people_detect
### 1.Prerequisites
* [ROS](http://wiki.ros.org/ROS/Installation)
* [jetson-inference](https://github.com/dusty-nv/jetson-inference)

### 2.Building workspace and compile
`mkdir -p ~/catkin_ws/src`<br>
next, copy this three packages to `/catkin_ws/src` and<br>
`catkin_make`<br>

## Usage 
* Set your own image topic

  find [ros_people_detect.launch](https://github.com/FanKaii/ros_people_detect/blob/master/people_detect/launch/ros_people_detect.launch) and [people_detect_test.launch](https://github.com/FanKaii/ros_people_detect/blob/master/people_detect_test/launch/people_detect_test.launch) and replace `/image_pub/image` with your own image topic.

* Run people_detect node

  `roslaunch people_detect ros_people_detect.launch`
  
* Run people_detect_test node

  `roslaunch people_detect_test people_detect_test.launch`
  
## Show result

* ![img1 load error](https://github.com/HeTingwei/ReadmeLearn/blob/master/avatar1.jpg)
  ![img1 load error](https://github.com/HeTingwei/ReadmeLearn/blob/master/avatar1.jpg)
