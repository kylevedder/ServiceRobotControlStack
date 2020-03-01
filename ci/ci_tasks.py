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

def run_build(compiler, optimization=''):
  run("rm -rf build/*", "Cleaning up build failed!")
  if compiler is None and optimization is None:
    run('catkin_make -j`nproc`', "Compilation with default config failed")
    return

  if compiler == "clang":
    cxx = "clang++"
    c = "clang"
  elif compiler == "gcc":
    cxx = "g++"
    c = "gcc"
  else:
    print("Unknown compiler: {}".format(compiler))
    exit(1)
  run('catkin_make -j`nproc` -DUSER_CXX_COMPILER="{}" -DUSER_C_COMPILER="{}" -DUSER_OPT_LEVEL="{}"'\
    .format(cxx, c, optimization), 
  "Compilation with CXX={}, C={}, OPT={} failed".format(cxx, c, optimization))


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
  sim_p = run_nonblock("rosrun control_stack simulator_node")
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

run('src/ServiceRobotControlStack/ci/format_all.sh && cd src/ServiceRobotControlStack && diff=$(git diff) && test -z "$diff" && cd ../../',
    "Code improperly formatted! Please run ./ci/format_all.sh")
run("src/ServiceRobotControlStack/ci/lint.sh", 
    "Lint failed!")

# Builds
run_build("clang")
run_build("clang", "-O3")
run_build("gcc")
run_build("gcc", "-O3")
run_build(None, None)

# Integration Tests
run_under_sim("rosrun control_stack nav_node", 
              "Starting nav_node failed!", 5)

valgrid_out_file = "valgrind_result.out"
run_under_sim('export ROS_HOME=`pwd` && roslaunch src/ServiceRobotControlStack/ci/launch_files/nav_node_valgrind.launch > {} 2>&1'.format(valgrid_out_file), 
              "Starting nav_node failed!", 30)
check_valigrind_out_file(valgrid_out_file)

              