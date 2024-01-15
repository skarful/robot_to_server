# robot_to_server repo

This repository subscribes to various sensor data and sends this data to a server via TCP and UDP. For this example, [Hong Kong dataset 5](https://github.com/weisongwen/UrbanLoco?tab=readme-ov-file#26-dataset-5-hk-data20190117) from UrbanLoco was used. 

## Structure
The repo consists of three parts.
- ros_transmitter package: This is a ROS C++ package that contains a separate socket client class. This is responsible for subscribing to sensor data and transmitting them. The Socket client class has two functions ```sendTCP()``` and ```sendUDP()```  to facilitate both TCP and UDP formats.
- novatel_msgs package: The messages from this format are used by the single enclosure GNSS developed by Novatel. As this has been deprecated and is not available for ROS Noetic, I have included it from the source.
- python_socket_server.py: A sample python socket server that can be run locally to test the ros_transmitter package.

## Design
Following design considerations have been made for this repo:
- Use of two ports (UDP and TCP) on the socket client allowing flexibility. TCP has been used for smaller data packets (such as IMU, GNSS) and UDP has been used for larger data (such as images and pointclouds).
- Singleton class for the socket client: As this class is responsible for opening and closing sockets, a singleton design is used. This allows multiple classes to use the same ports via the singleton instance.
- To ensure low latency, the message data is sent as and when the data is received in the callback. Further, ```ros::spin()``` was chosen in the main loop to allow the fastest loop/callback rate according to system architecture.
- Data is serialized into bytes using [roscpp serialization](https://wiki.ros.org/roscpp_serialization). This package uses template functions and is compatible with most ROS message types. 


## Requirements and Usage

This package runs on ROS Noetic, Ubuntu 20. There are no extra dependencies apart from those that are already installed with ROS Noetic desktop installation. A dockerfile has also been provided for ease of use on other platforms. 

The launch file contains IP address, TCP port and UDP port parameters that can be edited before launch. 

```bash
git clone <project name> #In your catkin_ws/src file
catkin build ros_transmitter
source devel/setup.bash

# Run server first in another terminal
cd src/robot_to_server
chmod +x python_socket_server.py
python3 python_socket_server.py

# Now launch the ros_transmitter node
roslaunch ros_transmitter ros_transmitter.launch
```

https://github.com/skarful/robot_to_server/assets/87267305/df6c6e58-d41b-4b34-869a-a43ce6e55e3d


## Future Improvements
- The python server side can be improved - presently we just check for the type of data received from the header and print this out. De-serialization/storage or processing of the received data can be added
- Present version does not have a re-connection provision in case communication is lost between server and client. This can be added. 
 
