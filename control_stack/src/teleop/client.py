#!/usr/bin/env python3
import socket

# create a socket object
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 

# get local machine name
host = socket.gethostname()                           
print(host)

port = 9001

# connection to hostname on the port.
s.connect(("127.0.0.1", port))     

s.send("robot0".encode('ascii'))

while True:
  # Receive no more than 1024 bytes
  msg = s.recv(1024)                                     

  print (msg.decode('ascii').strip())