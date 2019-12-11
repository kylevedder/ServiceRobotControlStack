#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D
import SimpleHTTPServer
import SocketServer
import socket
import signal
from subprocess import check_output


PORT = 8000
pub = rospy.Publisher('/nav_goal', Pose2D, queue_size=10)
rospy.init_node('goal_publisher', anonymous=True)
rate = rospy.Rate(100) # 10hz

named_locations = {"door" : [-1, -1, 0], "water": [-2, -7, 0], "middle": [-1.5, -3, 0]}

shutting_down = False
def signal_handler(signal, frame):
    print('Killing Server!')
    global shutting_down
    shutting_down = True
    httpd.server_close()

signal.signal(signal.SIGINT, signal_handler)

class ServerHandler(SimpleHTTPServer.SimpleHTTPRequestHandler):
    def server_bind(self):
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(self.server_address)

    def _set_headers(self):
        self.send_response(200)
        self.send_header("Content-type", "text/plain")
        self.end_headers()

    def _set_headers_fail(self):
        self.send_response(400)
        self.send_header("Content-type", "text/plain")
        self.end_headers()

    def _get_position_from_name(self, name):
        try:
            return named_locations[name.strip().lower()]
        except:
            return None

    def do_POST(self):
        content_len = int(self.headers.getheader('content-length', 0))
        post_body = self.rfile.read(content_len)
        position = self._get_position_from_name(post_body)
        if position is None:
            self._set_headers_fail()
            self.wfile.write("Unknown location: {}".format(post_body))
            return
        for _ in range(10):
            pub.publish(*position)
        self._set_headers()
        self.wfile.write("Set goal to {} {} {} ({})".format(position[0], position[1], position[2], post_body))

def get_ip():
    out = check_output(["ifconfig"])
    devices = out.split("\n\n")
    for d in devices:
        if "inet 10." in d:
            return d.split("inet ")[1].split(" netmask")[0].strip()
    raise ValueException

Handler = ServerHandler

httpd = SocketServer.TCPServer(("", PORT), Handler)
httpd.allow_reuse_address = True
ip = get_ip()
print("Server IP Address: {}:{}".format(ip, PORT))
try:
    httpd.serve_forever()
except socket.error as e:
    if shutting_down:
        pass
    else:
        raise e