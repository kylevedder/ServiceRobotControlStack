#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D
import SimpleHTTPServer
import SocketServer
import socket
import signal
from subprocess import check_output
out = check_output(["ifconfig"])

def get_ip(out):
    devices = out.split("\n\n")
    for d in devices:
        if "inet 10." in d:
            return d.split("inet ")[1].split(" netmask")[0].strip()
    raise ValueException

PORT = 8000
pub = rospy.Publisher('/nav_goal', Pose2D, queue_size=10)
rospy.init_node('goal_publisher', anonymous=True)
rate = rospy.Rate(100) # 10hz

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

    def do_POST(self):
        content_len = int(self.headers.getheader('content-length', 0))
        post_body = self.rfile.read(content_len)
        arr = post_body.split(' ')
        if len(arr) != 3:
            return
        arr = [float(e) for e in arr]
        print(arr)
        for _ in range(10):
            pub.publish(*arr)
        self._set_headers()
        self.wfile.write("Set goal to {} {} {}".format(*arr))

Handler = ServerHandler

httpd = SocketServer.TCPServer(("", PORT), Handler)
httpd.allow_reuse_address = True
ip = get_ip(out)
print("Server IP Address: {}:{}".format(ip, PORT))
try:
    httpd.serve_forever()
except socket.error as e:
    if shutting_down:
        pass
    else:
        raise e