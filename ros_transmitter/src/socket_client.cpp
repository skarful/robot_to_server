//----------------------------------------------------------------------------
//
//  Name              - socket_client.cpp
//
//  Description       - Functions part of SocketClient singleton class
//
//  Date (last edit)  - 14th Jan 2024
//----------------------------------------------------------------------------

#include <ros_transmitter/socket_client.h>

//Constructor
SocketClient::SocketClient(const std::string& ip, int tcp_port, int udp_port){

	//Initiate the TCP socket
	sockTCP = socket(AF_INET, SOCK_STREAM, 0);
	if (sockTCP == -1) {
		ROS_ERROR("Failed to initialize TCP client socket!");
		exit(EXIT_FAILURE);
	}

	sockaddr_in serv_addr;
	serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(tcp_port);

    //Check if address is valid
    if (inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr) <= 0) {
        ROS_ERROR("Chosen IP address not supported for TCP communication");
        exit(EXIT_FAILURE);
    }

    //Connect to server
    if ((connect(sockTCP, (struct sockaddr*)&serv_addr, sizeof(serv_addr))) < 0) {
        ROS_ERROR("Unable to connect to TCP server. Make sure it is up and retry. Exiting application");
        exit(EXIT_SUCCESS);
    }

    ROS_INFO_STREAM("Connected to TCP port "<< tcp_port);

    //Initiate UDP socket
    sockUDP = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockUDP == -1) {
		ROS_ERROR("Failed to initialize UDP client socket!");
		exit(EXIT_FAILURE);
	}

	memset(&servaddr, 0, sizeof(servaddr)); 
	servaddr.sin_family = AF_INET; 
    servaddr.sin_port = htons(udp_port); 
    
    //Check if address is valid
    if (inet_pton(AF_INET, ip.c_str(), &servaddr.sin_addr) <= 0) {
        ROS_ERROR("Chosen IP address not supported for UDP communication");
        exit(EXIT_FAILURE);
    }

    ROS_INFO_STREAM("UDP output port set to "<< udp_port);
    
}

//Send data via TCP
void SocketClient::sendTCP(const std::string& label, std::vector<uint8_t>& message){

	//Add the label as a prefix before sending data
	std::vector<uint8_t> prefixed_data(label.begin(), label.end());
    prefixed_data.insert(prefixed_data.end(), message.begin(), message.end());

    ROS_DEBUG_STREAM("data size:" <<prefixed_data.size() <<"data:" <<prefixed_data.data() );
    if (send(sockTCP, prefixed_data.data(), prefixed_data.size(), 0) == -1) {
        ROS_ERROR("Error in sending data via TCP");
        exit(EXIT_FAILURE);
    }
}


//Send data via UDP
void SocketClient::sendUDP(const std::string& label, std::vector<uint8_t>& message){

	//Add the label as a prefix before sending data [so server knows what data it is]
	std::vector<uint8_t> prefixed_data(label.begin(), label.end());
    prefixed_data.insert(prefixed_data.end(), message.begin(), message.end());

    ROS_DEBUG_STREAM("data size: " <<prefixed_data.size() <<"data: " <<prefixed_data.data() );

    size_t total_size = prefixed_data.size();
    size_t offset = 0;

    while (offset < total_size) {
        // Calculate the size of the current packet
        size_t packet_size = std::min(1024, int(total_size - offset));

        // Extract a packet from the large data
        std::vector<uint8_t> packet(prefixed_data.begin() + offset, prefixed_data.begin() + offset + packet_size);

        // Send the current packet
        int numt = sendto(sockUDP, packet.data(), packet.size()*sizeof(uint8_t), 0, (struct sockaddr*)&servaddr, sizeof(servaddr)); 

        // Update the offset for the next packet
        offset += packet_size;
    }
    
}