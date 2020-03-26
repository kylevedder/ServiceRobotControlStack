#!/usr/bin/env python3
import socket
import json

# create a socket object
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

url = "127.0.0.1"
port = 9001

# connection to hostname on the port.
try:
  s.connect((url, port))
except ConnectionRefusedError:
  exit("Failed to connect socket to command server at {}".format(url))

s.send("robot0".encode('ascii'))

while True:
  # Receive no more than 1024 bytes

  msg = s.recv(1024)
  msg_str = msg.decode('ascii').strip()
  if len(msg_str) == 0:
    # Socket connection ended.
    break
  try:
    msg_json = json.loads(msg_str)
  except json.decoder.JSONDecodeError:
    print("Failed to parse >>{}<<".format(msg_str))
    continue
  print(msg_json)

s.close()
