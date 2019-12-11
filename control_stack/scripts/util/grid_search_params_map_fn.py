import subprocess
import pandas
import numpy as np
import os.path

def run_pf(error_filename, param_filename):
  subprocess.run("rosrun particle_filter particle_filter_bag_reader {} src/particle_filter/config/sim_config.lua {}".format(error_filename, param_filename), shell=True, stdout=subprocess.DEVNULL)

def get_errors_centroid(filename):
  assert(os.path.isfile(filename))
  df = pandas.read_csv(filename)
  col = df["cent_error_norm"]
  return (col.quantile(0.05), col.quantile(0.5), col.quantile(0.95))

def get_errors_max(filename):
  assert(os.path.isfile(filename))
  df = pandas.read_csv(filename)
  col = df["max_error_norm"]
  return (col.quantile(0.05), col.quantile(0.5), col.quantile(0.95))

def remove_file(filename):
  subprocess.run("rm {}".format(filename), shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def gen_config_file(laser, arc, rot, consist):
  template = \
"""pf = {{
  kLaserStdDev = {};
  kArcStdDev = {};
  kRotateStdDev = {};
  kTemporalConsistencyWeight = {};
}};
""".format(laser, arc, rot, consist)
  return template

def write_config(filename, content):
  f = open(filename, 'w')
  f.write(content)
  f.close()

def gen_filenames(laser, arc, rot, consist):
  return ("/tmp/error_{}l{}a{}r{}c.lua".format(laser,arc,rot,consist), "/tmp/pf_config_{}l{}a{}r{}c.lua".format(laser,arc,rot,consist))

def map_fn(args):
  laser, arc, rot, consist = args
  print("Laser {} Arc {} Rot {} Consist {}".format(laser, arc, rot, consist))
  content = gen_config_file(laser, arc, rot, consist)
  error_filename, pf_config_filename = gen_filenames(laser, arc, rot, consist)
  write_config(pf_config_filename, content)
  remove_file(error_filename)
  run_pf(error_filename, pf_config_filename)
  centroid_5th, centroid_50th, centroid_95th = get_errors_centroid(error_filename)
  max_5th, max_50th, max_95th = get_errors_max(error_filename)
  return (laser, arc, rot, consist, centroid_5th, centroid_50th, centroid_95th, max_5th, max_50th, max_95th)