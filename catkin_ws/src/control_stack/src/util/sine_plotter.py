#!/usr/bin/env python3
import argparse
import csv
import math
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("true_pose_file")
args = parser.parse_args()
f = open(args.true_pose_file, 'r')
r = csv.reader(f)
numerical_rs = [[float(e) for e in l] for l in r]
vel_x, cmd_x, pose_x, stop_x = zip(*numerical_rs)

def compute_stop_x(p, v):
    acc = 1
    # vf^2 = vi^2 + 2a dx
    return np.sign(v) * (v * v) / (2 * acc) + p

comp_stop_x = [compute_stop_x(p, v) for p, v in zip(pose_x, cmd_x)]


import matplotlib.pyplot as plt
plt.plot(range(len(pose_x)), pose_x, label="CURR POSE")
plt.plot(range(len(stop_x)), stop_x, label="STOP POSE")
plt.plot(range(len(comp_stop_x)), comp_stop_x, label="COMP STOP POSE")
plt.plot(range(len(vel_x)), vel_x, label="VEL")
plt.plot(range(len(cmd_x)), cmd_x, label="CMD")

for i in range(math.ceil(len(cmd_x) / 160)):
    plt.axvline(i * 160, color='black')
    plt.axvline(i * 160 + 80, color='red')
plt.legend()
plt.xlabel("Tick")
plt.ylabel("Position (m)")
plt.show()
