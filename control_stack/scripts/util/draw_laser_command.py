#!/usr/bin/env python3
import sys
import numpy as np
import joblib
import matplotlib.pyplot as plt
from matplotlib import collections  as mc

def parse_args():
    if len(sys.argv) != 7:
        print("Usage:", sys.argv[0], "[path to laser] [path to command] [path to dynamic map] [path to trajectories] [path to true command] [path to loss positions]")
        exit(-1)
    return sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5] , sys.argv[6]

laser_file, command_file, dynamic_map_file, trajectories_file, true_command_file, loss_positions_file = parse_args()

laser_data = np.loadtxt(laser_file)
laser_data_xs = np.expand_dims(laser_data[:, 0::2], axis=2)
laser_data_ys = np.expand_dims(laser_data[:, 1::2], axis=2)
laser_data = np.concatenate([laser_data_xs, laser_data_ys, np.zeros(laser_data_ys.shape)], axis=2)
print(laser_data.shape)

try:
    command_data = np.loadtxt(command_file)
except UnicodeDecodeError:
    command_data = joblib.load(command_file)

command_data_mag = command_data[:, 0::2]
command_data_ang = command_data[:, 1::2]
command_data_cos = np.cos(command_data_ang)
command_data_sin = np.sin(command_data_ang)
command_data = np.concatenate([command_data_mag, command_data_cos, command_data_sin], axis=1)

try:
    true_command_data = np.loadtxt(true_command_file)
except UnicodeDecodeError:
    true_command_data = joblib.load(true_command_file)

true_command_data_mag = true_command_data[:, 0::2]
true_command_data_ang = true_command_data[:, 1::2]
true_command_data_cos = np.cos(true_command_data_ang)
true_command_data_sin = np.sin(true_command_data_ang)
true_command_data = np.concatenate([true_command_data_mag, true_command_data_cos, true_command_data_sin], axis=1)

kUseMaps = False

if kUseMaps:
    maps = [eval(l) for l in open(dynamic_map_file, 'r').readlines()]
trajectories = [eval(l) for l in open(trajectories_file, 'r').readlines()]
print(len(trajectories))

loss_positions = [eval(l) for l in open(loss_positions_file, 'r').readlines()]

def draw_trajectory(tr):
    """ ss << "(" << tr.final_pose.tra.x() << ", " << tr.final_pose.tra.y()
         << ", " << tr.final_pose.rot << ", " << tr.rotate_circle_center.x()
         << ", " << tr.rotate_circle_center.y() << ", "
         << tr.rotate_circle_radius << ", " << tr.achieved_vel_pose.tra.x()
         << ", " << tr.achieved_vel_pose.tra.y() << ", "
         << tr.achieved_vel_pose.rot << "), ";"""
    for t in tr:
        ax = plt.gca()
        ax.add_artist(plt.Circle((t[0], t[1]), 0.3, color='red', alpha=0.1))
        ax.add_artist(plt.Circle((t[0], t[1]), 0.01, color='red', alpha=1))
        ax.add_artist(plt.Circle((t[3], t[4]), t[5], color='green', alpha=0.1))
        ax.add_artist(plt.Circle((t[6], t[7]), 0.3, color='blue', alpha=0.1))


def draw_loss_positions(lps):
    for lp in lps:
        ax = plt.gca()
        ax.add_artist(plt.Circle((lp[0], lp[1]), 0.2, color='orange', alpha=0.5))
        plt.text(lp[0], lp[1], str(lp[2]))


for i in range(laser_data.shape[0]):
    l = laser_data[i]
    c = command_data[i]
    t = trajectories[i]
    tc = true_command_data[i]
    lp = loss_positions[i]

    draw_loss_positions(lp)

    if kUseMaps:
        m = maps[i]
        lines = [[(x1, y1), (x2, y2)] for x1, y1, x2, y2 in m]
        lc = mc.LineCollection(lines)
        plt.gca().add_collection(lc)


    command_forward = (c[0]*c[3], c[0]*c[6])
    command_left = (c[1]*c[4], c[1]*c[7])
    command_right = (c[2]*c[5], c[2]*c[8])

    tc_command_forward = (tc[0]*tc[3], tc[0]*tc[6])
    tc_command_left = (tc[1]*tc[4], tc[1]*tc[7])
    tc_command_right = (tc[2]*tc[5], tc[2]*tc[8])

    if c[0] > 0:
        plt.arrow(0, 0, c[0]*c[3], c[0]*c[6], color='black')
    if c[1] > 0:
        plt.arrow(0, 0, c[1]*c[4], c[1]*c[7], color='blue')
    if c[2] > 0:
        plt.arrow(0, 0, c[2]*c[5], c[2]*c[8], color='green')
    plt.scatter(l[:, 0], l[:, 1], s=0.1, color='green', alpha=0.2)
    draw_trajectory(t)
    plt.gca().set_aspect('equal', 'box')
    plt.ylim(-2, 2)
    plt.xlim(-1, 3)
    plt.title('img{:05d}.png\nForward {:.4f}, {:.4f} Left {:.4f}, {:.4f} Right: {:.4f}, {:.4f}\nForward {:.4f}, {:.4f} Left {:.4f}, {:.4f}  Right: {:.4f}, {:.4f}'.format(i, *command_forward, *command_left, *command_right, *tc_command_forward, *tc_command_left, *tc_command_right))
    plt.savefig('img{0:05d}.png'.format(i))
    plt.clf()
    print(i)
# f = open(map_file, 'r')
# lines = f.readlines()
# print("Read", len(lines), "walls")
# for l in lines:
#     elements = [float(e) for e in l.split(' ')]
#     if len(elements) != 4:
#         print("Improper number coordinates given:", l)
#         exit(-1)
#     plt.plot([elements[0], elements[2]], [elements[1], elements[3]])
# plt.gca().set_aspect('equal', 'box')
# plt.show()
    