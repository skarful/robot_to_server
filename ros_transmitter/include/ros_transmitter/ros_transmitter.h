//----------------------------------------------------------------------------
//
//  Name              - ros_transmitter.h
//
//  Description       - Consists of a class that subscribes to various topics and sends the received data to a server via TCP/UDP
//						Uses the socket_client class to serialize and send the data
//
//  Date (last edit)  - 14th Jan 2024
//----------------------------------------------------------------------------

#ifndef ROS_TRANSMITTER_H
#define ROS_TRANSMITTER_H


#include <ros_transmitter/socket_client.h>
#include <ros/ros.h>

//Include sensor message types
#include <sensor_msgs/NavSatFix.h>
#include <novatel_msgs/INSPVAX.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

class Transmitter{

public:

	//Constructor
	Transmitter(const std::string& ip_address, int TCP_port, int UDP_port);

	//Serializer 
	template <class M>
    std::vector<uint8_t> serializeMessage(const M& msg);

private:

	//ROS nodehandle
	ros::NodeHandle nh; 

	//Callback function prototypes for subscribed topics
	void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg); 
	void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
	void cameraCallback(const sensor_msgs::Image::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void spanCallback(const novatel_msgs::INSPVAX::ConstPtr& msg);

	//ROS subscribers for required topics
	ros::Subscriber gnss_sub = nh.subscribe<sensor_msgs::NavSatFix>("/ublox_gps_node/fix" , 1, &Transmitter::gnssCallback,this);                
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_0" , 1, &Transmitter::lidarCallback,this);
    ros::Subscriber camera_sub = nh.subscribe<sensor_msgs::Image>("/camera/image_color" , 1, &Transmitter::cameraCallback,this);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data" , 1, &Transmitter::imuCallback,this);
	ros::Subscriber span_sub = nh.subscribe<novatel_msgs::INSPVAX>("/novatel_data/inspvax" , 1, &Transmitter::spanCallback,this);    

};

#endif