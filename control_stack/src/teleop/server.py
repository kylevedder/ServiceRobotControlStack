#!/usr/bin/env python3

from flask import Flask, request, send_from_directory, render_template
import socket                                         
import threading
import time
import signal
import json

kTimeout = 0.1
socket.setdefaulttimeout(kTimeout)
robot_connection_map = dict()

is_running = True

# create a socket object
serversocket = socket.socket(
	        socket.AF_INET, socket.SOCK_STREAM)
serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

host = "192.168.0.102"

port = 9001

def thread_function():
    while is_running:
      try:
        clientsocket, addr = serversocket.accept()
      except:
        continue
      msg = clientsocket.recv(1024)
      decoded_msg = msg.decode('ascii')
      print("Got a connection from {} for {}".format(str(addr), decoded_msg))
      robot_connection_map[decoded_msg] = (clientsocket, addr)

x = threading.Thread(target=thread_function)

def cleanup():
    global is_running
    is_running = False
    time.sleep(kTimeout)
    x.join()
    for k in robot_connection_map:
        clientsocket, addr = robot_connection_map[k]
        clientsocket.close()
    serversocket.close()

def setup_sockets():
    # bind to the port
    serversocket.bind((host, port))

    # queue up to 5 requests
    serversocket.listen(5)

    x.start()

if __name__ == "__main__":
    setup_sockets()

    app = Flask(__name__, static_url_path='')

    kCommandEndpoint = "/send_command"

    @app.route(kCommandEndpoint, methods=['POST'])
    def result():
        data = request.json
        robot_name = data['robot']
        res = robot_connection_map.get(robot_name, None)
        if not res:
            return "Failure!"
        clientsocket, addr = res
        clientsocket.send(json.dumps(data).encode('ascii'))
        return "Success!"

    @app.route('/')
    def root():
        return app.send_static_file('index.html')

    @app.route('/robots/<string:robot_name>')
    def robot_page(robot_name):
        return render_template('robot_control_template.html',
                               robot_name=robot_name,
                               server_url=kCommandEndpoint)


    app.run(host, debug=False, port=5000)
    cleanup()