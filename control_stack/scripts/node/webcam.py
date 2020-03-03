#!/usr/bin/env python3
import cv2
import signal
import sys
import joblib
import pickle
import requests
import sys
import argparse
from timeit import default_timer as timer

parser = argparse.ArgumentParser()
parser.add_argument("--ip", type=str, default="localhost", help="Server hostname")
parser.add_argument("--camera_index", type=int, default=0, help="Connected camera index")
opt = parser.parse_args()

url = "http://" + opt.ip

def sigint_handler(signal, frame):
    print('Interrupted')
    sys.exit(0)
    cap.release()
    cv2.destroyAllWindows()
signal.signal(signal.SIGINT, sigint_handler)
cap = cv2.VideoCapture(opt.camera_index)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)
previous_time = 0
while True:
    current_time = timer()
    ret, frame = cap.read()

    delta = (current_time - previous_time)
    if frame is None or  delta < 1:
        continue
    print(delta)
    print("Sending image...")
    previous_time = current_time
    try:
        r = requests.post(url, data = pickle.dumps(frame))
        if r is None or r.status_code != 200:
             print("POST failed")
             continue
        print(r.text)
    except Exception as e:
        print(e)
        
cap.release()
