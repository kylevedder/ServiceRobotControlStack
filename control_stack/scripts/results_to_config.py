#!/usr/bin/env python3

import subprocess
import pandas
import numpy as np
import sys

def get_error():
  df = pandas.read_csv(sys.argv[1])
  return df.loc[df['max_95th'].idxmin()], df.loc[df['centroid_95th'].idxmin()]

max_percentile, cent_percentile = get_error()
print(max_percentile)
print(cent_percentile)