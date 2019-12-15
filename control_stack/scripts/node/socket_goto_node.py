#!/usr/bin/env python
import socket
import sys
import rospy
from geometry_msgs.msg import Pose2D

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Bind the socket to the port
server_address = ('10.103.118.91', 8000)
print >>sys.stderr, 'starting up on %s port %s' % server_address
sock.bind(server_address)
# Listen for incoming connections
sock.listen(1)
pub = rospy.Publisher('/nav_goal', Pose2D, queue_size=10)
rospy.init_node('goal_publisher', anonymous=True)
rate = rospy.Rate(100) # 10hz

"""
starting up on 10.103.118.91 port 8000
waiting for a connection
connection from ('10.103.95.125', 43562)
received "{ "skill_name" : "Go_to","location": "water"}"
sending data back to the client
received ""
no more data from ('10.103.95.125', 43562)
waiting for a connection
"""

named_locations = {"door" : [-1, -1, 0], "water": [-2, -7, 0], "middle": [-1.5, -3, 0]}
def get_position_from_name(name):
    try:
        return named_locations[name.strip().lower()]
    except:
        return None

while True:
    # Wait for a connection
    print >>sys.stderr, 'waiting for a connection'
    connection, client_address = sock.accept()
    try:
        print >>sys.stderr, 'connection from', client_address

        # Receive the data in small chunks and retransmit it
        while True:
            data = connection.recv(4000)
            if data:
                name = data.split('"location": ')[1].replace("}", "").replace('"', '').strip()
                print(name)
                pos = get_position_from_name(name)
                if pos is not None:
                    for _ in range(10):
                        pub.publish(*pos)
            else:
                print >>sys.stderr, 'no more data from', client_address
                break
            
    finally:
        # Clean up the connection
        connection.close()