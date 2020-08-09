#!/usr/bin/env python3
import subprocess
import time
import os

def fail(msg, quit):
  print(msg)
  if quit:
    exit(1)

def run(cmd, msg, quit=True, timeout=None):
  try:
    print(">>>>>>>>>" + cmd)
    cp = subprocess.run(cmd, shell=True, check=True, timeout=timeout)
    print("<<<<<<<<<" + cmd)
    return cp.returncode == 0
  except subprocess.TimeoutExpired:
    return True
  except:
    print("<<<<<<<<<" + cmd)
    fail(msg, quit)
    return False


def run_nonblock(cmd):
  return subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL)

def kill_proc(p):
  try:
    os.killpg(os.getpgid(p.pid), subprocess.signal.SIGINT)
  except:
    pass


def run_under_sim(cmd, msg, timeout):
  roscore_p = run_nonblock("roscore")
  time.sleep(4)
  sim_p = run_nonblock("./rosbuild_ws/simulator/ut_multirobot_sim/bin/simulator --sim_config=rosbuild_ws/simulator/sim_config.lua")
  time.sleep(1)
  succeeded = run(cmd, msg, quit=False, timeout=timeout)
  kill_proc(sim_p)
  kill_proc(roscore_p)
  if not succeeded:
    exit(1)
  
def check_valigrind_out_file(filename):
  with open(filename) as f:
    ls = f.readlines()
    for l in ls:
      for s in ["uninitialised", "invalid read"]:
        if s in l:
          print("Valgring errors detected!")
          print("\n".join(ls))
          exit(1)

run("ci/lint.sh", "Lint failed!")
# run('ci/format_all.sh && diff=$(git diff) && test -z "$diff"',
#     "Code improperly formatted! Please run ./ci/format_all.sh")

# Integration Tests
run_under_sim("cd catkin_ws/ && devel/lib/control_stack/nav_node src/control_stack/config/sim_config.lua && cd ..", 
              "Starting nav_node failed!", 5)

valgrid_out_file = "valgrind_result.out"
run_under_sim("cd catkin_ws/ && valgrind devel/lib/control_stack/nav_node src/control_stack/config/sim_config.lua > {} 2>&1".format(valgrid_out_file), 
              "Starting nav_node failed!", 30)
check_valigrind_out_file(valgrid_out_file)

              