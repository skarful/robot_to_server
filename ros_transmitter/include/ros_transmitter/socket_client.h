 //----------------------------------------------------------------------------
//
//  Name              - socket_client.h
//
//  Description       - A socket client class that sends data to a server via UDP or TCP. Note: this class is a singleton class
//
//  Date (last edit)  - 14th Jan 2024
//----------------------------------------------------------------------------

#ifndef SOCKET_CLIENT_H
#define SOCKET_CLIENT_H

#include <arpa/inet.h>
#include <unistd.h> 
#include <netinet/in.h>
#include <string>
#include <ros/ros.h>
#include <algorithm>

class SocketClient{

public:

	//Get instance of class
	static SocketClient& getInstance(const std::string& ip="", int tcp_port=0, int udp_port=0) {
        static SocketClient instance(ip, tcp_port, udp_port);
        return instance;
    }

    void sendTCP(const std::string& label, std::vector<uint8_t>& message);
    void sendUDP(const std::string& label, std::vector<uint8_t>& message);


private:

	//Private constructor
	SocketClient(const std::string& ip, int tcp_port, int udp_port);

	//Private variables
	int sockTCP;
	int sockUDP;

	//Required to store this for UDP
	sockaddr_in servaddr;

};

#endif