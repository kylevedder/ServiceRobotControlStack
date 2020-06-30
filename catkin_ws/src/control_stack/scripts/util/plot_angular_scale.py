#!/usr/bin/env python3

f1 = open("onethirdfile.txt", 'r').readlines()[0].split(",")[:-1]
f2 = open("twothirdfile.txt", 'r').readlines()[0].split(",")[:-1]
f3 = open("threethirdfile.txt", 'r').readlines()[0].split(",")[:-1]

print(f1)

import matplotlib.pyplot as plt
import numpy as np


def plot(data):
    data = [float(e) for e in data]
    plt.plot(np.linspace(0, data[0], 100), data[2:])

plot(f1)
plot(f2)
plot(f3)
plt.show()

