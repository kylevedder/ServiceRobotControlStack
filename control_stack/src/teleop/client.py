#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState
import requests
import socket
import json
import argparse
import threading
import signal

kTimeout = 0.1
socket.setdefaulttimeout(kTimeout)
parser = argparse.ArgumentParser()
parser.add_argument("robot_name", help="Robot name [robot0|robot1|robot2]")
parser.add_argument("server_url", help="Robot url without protocol")
opt = parser.parse_args()

url = opt.server_url
port = 9001

def sensor_state_callback(sensor_state):
  if sensor_state.header.seq % 20 != 0:
    return
  is_charging = (sensor_state.charger != 0)
  battery = sensor_state.battery
  data = {'robot' : opt.robot_name, 'is_charging' : is_charging, 'battery' : battery}
  requests.post(url= "http://" + url + '/update_status', data = data)



rospy.init_node('teleop')
teleop_pub = rospy.Publisher('/teleop_topic', Twist, queue_size=10)
status_sub = rospy.Subscriber('/mobile_base/sensors/core', SensorState)


# create a socket object
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# connection to hostname on the port.
try:
  s.connect((url, port))
except socket.error:
  exit("Failed to connect socket to command server at {}".format(url))

s.send(opt.robot_name.encode('ascii'))
is_running = True

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

def socket_reading_thread():
  while is_running:
    try:
      # Receive no more than 1024 bytes
      msg = s.recv(1024)
    except:
      continue
    msg_str = msg.decode('ascii').strip()
    if len(msg_str) == 0:
      # Socket connection ended.
      print("Socket connect to command server terminated!")
      break
    try:
      msg_json = json.loads(msg_str)
    except:
      print("Failed to parse >>{}<<".format(msg_str))
      continue
    print(msg_json)
    teleop_pub.publish(make_twist(msg_json))

x = threading.Thread(target=socket_reading_thread)
x.start()

def handler(num, frame):
  global  is_running
  is_running = False
  x.join()
  s.close()

signal.signal(signal.SIGINT, handler)

rospy.spin()
