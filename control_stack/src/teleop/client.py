#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist
import socket
import json
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("robot_name", help="Robot name [robot0|robot1|robot2]")
opt = parser.parse_args()


rospy.init_node('teleop')
pub = rospy.Publisher('/teleop_topic', Twist, queue_size=10)

# create a socket object
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

url = "127.0.0.1"
port = 9001

# connection to hostname on the port.
try:
  s.connect((url, port))
except socket.error:
  exit("Failed to connect socket to command server at {}".format(url))

s.send(opt.robot_name.encode('ascii'))

def make_twist(msg_json):
  delta_x = 0
  delta_theta = 0

  kForwardDelta = 0.1
  kThetaDelta = 0.5

  if msg_json[u'forward']:
    delta_x += kForwardDelta
  if msg_json[u'backward']:
    delta_x -= kForwardDelta
  if msg_json[u'left']:
    delta_theta += kThetaDelta
  if msg_json[u'right']:
    delta_theta -= kThetaDelta

  twist = Twist()
  twist.linear.x = delta_x
  twist.angular.z = delta_theta
  return twist

while True:
  # Receive no more than 1024 bytes
  msg = s.recv(1024)
  msg_str = msg.decode('ascii').strip()
  if len(msg_str) == 0:
    # Socket connection ended.
    print("Socket connect to command server terminated!")
    break
  try:
    msg_json = json.loads(msg_str)
  except json.decoder.JSONDecodeError:
    print("Failed to parse >>{}<<".format(msg_str))
    continue
  print(msg_json)
  pub.publish(make_twist(msg_json))

s.close()
