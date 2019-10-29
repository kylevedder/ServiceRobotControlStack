#!/usr/bin/env python3

import numpy as np
import itertools
import multiprocessing as mp
from functools import reduce
import grid_search_params_map_fn

def result_header(filename):
  f = open(filename, 'w')
  f.write("laser,arc,rot,consist,centroid_5th,centroid_50th,centroid_95th,max_5th,max_50th,max_95th\n")
  f.close()

def result_append(laser, arc, rot, consist, centroid_5th, centroid_50th, centroid_95th, max_5th, max_50th, max_95th):
  f.write("{},{},{},{},{},{},{},{},{},{}\n".format(laser, arc, rot, consist, centroid_5th, centroid_50th, centroid_95th, max_5th, max_50th, max_95th))

laser_vals = np.arange(0.001, 0.5, 0.05)
arc_vals = np.arange(0.001, 0.5, 0.05)
rot_vals = np.arange(0.001, 0.2, 0.02)
consist_vals = np.arange(0.0, 1.2, 0.075)


tasks = list(itertools.product(*[laser_vals, arc_vals, rot_vals, consist_vals]))

result_filename = "results.csv"
result_header(result_filename)

count = mp.cpu_count()
pool = mp.Pool(processes=count)

results = pool.map(grid_search_params_map_fn.map_fn, tasks)
print("Map completed, merging results")
f = open(result_filename, 'a')
for result in results:
  result_append(*result)
f.close()
