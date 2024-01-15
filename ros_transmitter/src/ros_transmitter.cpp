//----------------------------------------------------------------------------
//
//  Name              - ros_transmitter.cpp
//
//  Description       - Consists of Transmitter class functions including constructor
//
//  Date (last edit)  - 14th Jan 2024
//----------------------------------------------------------------------------

 #include <ros_transmitter/ros_transmitter.h>


//Constructor
Transmitter::Transmitter(const std::string& ip_address, int tcp_port, int udp_port){

	//First time setup of the SocketClient instance
	SocketClient& socketClient = SocketClient::getInstance(ip_address, tcp_port, udp_port);
	ROS_INFO("Transmitter object has been instatiated successfully");

}

//Template function to serialize ros messages
template <class M>
std::vector<uint8_t> Transmitter::serializeMessage(const M& msg) {
    uint32_t serial_size = ros::serialization::serializationLength(msg);
    std::vector<uint8_t> buffer(serial_size);
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    ros::serialization::serialize(stream, msg);
    return buffer;
}

//GNSS Callback Function
void Transmitter::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){

	std::vector<uint8_t> serialized_msg = serializeMessage(*msg);
	std::string label = "GNSS";

	//Send via TCP
	SocketClient& socketClient = SocketClient::getInstance();
	socketClient.sendTCP(label, serialized_msg);

	ROS_INFO("Sent GNSS Message");
}

//Lidar Callback Function
void Transmitter::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg){

	std::vector<uint8_t> serialized_msg = serializeMessage(*msg);
	std::string label = "LIDR";

	//Send via UDP
	SocketClient& socketClient = SocketClient::getInstance();
	socketClient.sendUDP(label, serialized_msg);

	ROS_INFO("Sent Lidar Message");
}

//Camera Callback Function
void Transmitter::cameraCallback(const sensor_msgs::Image::ConstPtr& msg){

	std::vector<uint8_t> serialized_msg = serializeMessage(*msg);
	std::string label = "CMRA";

	//Send via UDP
	SocketClient& socketClient = SocketClient::getInstance();
	socketClient.sendUDP(label, serialized_msg);

	ROS_INFO("Sent Camera Message");
}

//IMU Callback Function
void Transmitter::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){

	std::vector<uint8_t> serialized_msg = serializeMessage(*msg);
	std::string label = "IMUS";

	//Send via TCP
	SocketClient& socketClient = SocketClient::getInstance();
	socketClient.sendTCP(label, serialized_msg);

	ROS_INFO("Sent IMU Message");
}

//SPAN Callback Function
void Transmitter::spanCallback(const novatel_msgs::INSPVAX::ConstPtr& msg){

	std::vector<uint8_t> serialized_msg = serializeMessage(*msg);
	std::string label = "SPAN";

	//Send via TCP
	SocketClient& socketClient = SocketClient::getInstance();
	socketClient.sendTCP(label, serialized_msg);

	ROS_INFO("Sent SPAN message");
}