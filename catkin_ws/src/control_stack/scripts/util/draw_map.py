#!/usr/bin/env python3
import sys
import matplotlib.pyplot as plt

def parse_args():
    if len(sys.argv) != 2:
        print("Usage:", sys.argv[0], "[path to .map]")
        exit(-1)
    return sys.argv[1]

map_file = parse_args()

f = open(map_file, 'r')
lines = f.readlines()
print("Read", len(lines), "walls")
for l in lines:
    elements = [float(e) for e in l.split(',')]
    if len(elements) != 4:
        print("Improper number coordinates given:", l)
        exit(-1)
    plt.plot([elements[0], elements[2]], [elements[1], elements[3]])
plt.gca().set_aspect('equal', 'box')
plt.show()
    
