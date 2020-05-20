#!/usr/bin/env python3
import cv2
import queue
import threading
import time
import signal
import sys
import joblib
import pickle
import requests
import sys
import argparse
from timeit import default_timer as timer

parser = argparse.ArgumentParser()
parser.add_argument("--ip", type=str, default="localhost:5000", help="Server hostname")
parser.add_argument("--camera_index", type=int, default=0, help="Connected camera index")
opt = parser.parse_args()

class VideoCapture:
    def __init__(self, name):
        self.cap = cv2.VideoCapture(name)
        self.q = queue.Queue()
        t = threading.Thread(target=self._reader)
        t.daemon = True
        t.start()

    # read frames as soon as they are available, keeping only most recent one
    def _reader(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            if not self.q.empty():
                try:
                    self.q.get_nowait()   # discard previous (unprocessed) frame
                except Queue.Empty:
                    pass
            self.q.put(frame)

    def read(self):
        return self.q.get()


url = "http://" + opt.ip


def sigint_handler(signal, frame):
    print('Interrupted')
    sys.exit(0)
    cap.release()
    cv2.destroyAllWindows()
signal.signal(signal.SIGINT, sigint_handler)
cap = VideoCapture(opt.camera_index)
previous_time = 0
while True:
    current_time = timer()
    frame = cap.read()

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
