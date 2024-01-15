//----------------------------------------------------------------------------
//
//  Name              - main.cpp
//
//  Description       - This package consists of a ROS node that subscribes to certain topics and sends them to a server via TCP or UDP
//
//  Revision History  - NAME          DATE               DESCRIPTION 
//		  1.0	       Sukumar     14th Jan 2024       Initial version 
//----------------------------------------------------------------------------

#include <ros_transmitter/ros_transmitter.h>

int main(int argc,char** argv){

	ros::init(argc, argv, "sub_and_send");
	ros::NodeHandle nh; 

	//Default values
	std::string ip = "127.0.0.1";
	int tcp_port = 8080;
	int udp_port = 8081;

	//Read IP and port numbers from parameter server
	nh.getParam("/ros_transmitter/IP_Address", ip);
	nh.getParam("/ros_transmitter/TCP_Port", tcp_port);
	nh.getParam("/ros_transmitter/UDP_Port", udp_port);

	//Instantiate Transmitter class which contains SocketClient class
	Transmitter transmitter(ip, tcp_port, udp_port);

	while (ros::ok())
    {

    	//Let ROS spin at rate of callbacks
        ros::spin();
    }

	return 0;
}