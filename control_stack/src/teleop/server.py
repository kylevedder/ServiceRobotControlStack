#!/usr/bin/env python3

from flask import Flask, request, send_from_directory, render_template
import socket                                         
import threading
import time
import signal

socket.setdefaulttimeout(0.5)
robot_connection_map = dict()

is_running = True

# create a socket object
serversocket = socket.socket(
	        socket.AF_INET, socket.SOCK_STREAM) 

# get local machine name
host = "127.0.0.1" #socket.gethostname()                           

port = 9001

# # bind to the port
# serversocket.bind((host, port))                                  

# # queue up to 5 requests
# serversocket.listen(5)          

# def thread_function():
#     while is_running:
#       try:
#         clientsocket, addr = serversocket.accept()
#       except:
#         continue
#       msg = clientsocket.recv(1024)
#       decoded_msg = msg.decode('ascii')
#       print("Got a connection from {} for {}".format(str(addr), decoded_msg))
#       robot_connection_map[decoded_msg] = (clientsocket, addr)


# x = threading.Thread(target=thread_function)
# x.start()

# def cleanup():
#     global is_running
#     is_running = False
#     for k in robot_connection_map:
#         clientsocket, addr = robot_connection_map[k]
#         clientsocket.close()
#     x.join()
#     serversocket.close()


# def handler(signum, frame):
#     cleanup()

# # signal.signal(signal.SIGINT, handler)



# while True:
#     if not is_running:
#       break
#     time.sleep(0.2)
#     print(i)
#     to_be_deleted = []
#     for k in robot_connection_map:
#         clientsocket, addr = robot_connection_map[k]
#         msg = 'Iter '+ str(i) + "\n"
#         if not is_running:
#             break
#         try:
#           clientsocket.send(msg.encode('ascii'))
#         except BrokenPipeError:
#           to_be_deleted.append(k)
#     for k in to_be_deleted:
#       clientsocket, addr = robot_connection_map[k]
#       print("Lost a connection from {} for {}".format(str(addr), k))
#       del robot_connection_map[k]

# cleanup()

app = Flask(__name__, static_url_path='')

@app.route('/send_command', methods=['POST'])
def result():
    return "Success!"

@app.route('/')
def root():
    return app.send_static_file('index.html')

@app.route('/robots/<string:robot_name>')
def robot_page(robot_name):
    return render_template('robot_control_template.html', robot_name=robot_name)


app.run(host, debug=True, port=5000)