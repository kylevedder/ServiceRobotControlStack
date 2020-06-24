#!/usr/bin/env python3
import numpy as np
import argparse
import csv

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("in_file", help="Input file")
    parser.add_argument("out_file", help="Output file")
    parser.add_argument("old_scale", help="Input scale", type=float)
    parser.add_argument("new_scale", help="Output scale", type=float)
    args = parser.parse_args()
    return args.in_file, args.out_file, args.old_scale, args.new_scale

in_file, out_file, old_scale, new_scale = get_args()

in_file = open(in_file, 'r')
out_file = open(out_file, 'w')
for row in csv.reader(in_file):
    row = [str(float(e) / old_scale * new_scale) for e in row]
    out_file.write(', '.join(row) + '\n')
in_file.close()
out_file.close()