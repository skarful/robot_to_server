"""
Description:
Python Socket Server with TCP and UDP

Functionality:
- Listens to both TCP and UDP clients on specified IP, UDP port, and TCP port.
- Handles TCP connections in separate threads.
- Receives UDP datagrams in the main thread.

Author: Sukumar
Date: 15/1/2024
"""

import socket
import threading
import sys
import struct

def handle_tcp_client(client_socket):

    while True:

        type_bytes = client_socket.recv(4)
        if len(type_bytes) != 4:
            print("Error: Failed to read message type.")
            return None

        print(f"Received TCP data: {type_bytes.decode('utf-8')}")
        message = type_bytes.decode('utf-8')
        if message == "IMUS":
            message_size = 316
        elif message == "GNSS":
            message_size = 119
        elif message == "SPAN":  # SPAN
            message_size = 150
        else:
            print("Error: Unknown message type")
            return None

        # Read the remaining bytes of the message
        data_bytes = client_socket.recv(message_size)
        if len(data_bytes) != message_size:
            print(len(data_bytes))
            print("Error: Failed to read message data.")
            return None

        # Can add deserialization/ storage /processing code here

def main():
    if len(sys.argv) != 4:
        print("Usage: python python_socket_server.py <IP Address> <TCP_port> <UDP_port>")
        sys.exit(1)

    ip = sys.argv[1]
    tcp_port = int(sys.argv[2])
    udp_port = int(sys.argv[3])

    # Setup TCP server
    tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp_server.bind((ip, tcp_port))
    tcp_server.listen(5)
    print(f"TCP server listening on {ip}:{tcp_port}")

    # Setup UDP server
    udp_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_server.bind((ip, udp_port))
    print(f"UDP server listening on {ip}:{udp_port}")

    try:
        #Offload TCP connections to a new thread
        tcp_client, tcp_addr = tcp_server.accept()
        print(f"Connected by {tcp_addr}")
        tcp_thread = threading.Thread(target=handle_tcp_client, args=(tcp_client,))
        tcp_thread.start()

        # Receive UDP datagrams in the main thread
        while True:
            data, addr = udp_server.recvfrom(1024)
            print("Received UDP Data: Lidar packet")
            
    except KeyboardInterrupt:
        print("Server shutting down.")

    finally:
        # Close both sockets when the server is done
        tcp_server.close()
        #udp_server.close()

if __name__ == "__main__":
    main()