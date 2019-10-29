#!/usr/bin/env python3
import sys
import matplotlib.pyplot as plt
import pandas

def parse_args():
    if len(sys.argv) != 3:
        print("Usage:", sys.argv[0], "[path to mything.csv] [path to standard.csv]")
        exit(-1)
    return (sys.argv[1], sys.argv[2])

mything_csv_file, standard_csv_file = parse_args()

mything_df = pandas.read_csv(mything_csv_file)
standard_df = pandas.read_csv(standard_csv_file)
#print(mything_df)
plt.subplot(211)
plt.title("Weighted Average Particle Estimate")
row_name = "cent_error_norm"
print("{}% error of standard".format(mything_df[row_name].mean() / standard_df[row_name].mean() * 100))
plt.plot(range(len(mything_df[row_name])), mything_df[row_name], 'g', label="PF + Short-term features")
plt.plot(range(len(mything_df[row_name])), [mything_df[row_name].mean()] * len(mything_df[row_name]), 'g')
plt.plot(range(len(standard_df[row_name])), standard_df[row_name], 'r', label="Standard PF")
plt.plot(range(len(standard_df[row_name])), [standard_df[row_name].mean()] * len(standard_df[row_name]), 'r')
plt.xlabel("Step")
plt.ylabel("Translational error (meters)")
plt.legend()
plt.grid()
plt.subplot(212)
plt.title("Max Weighted Particle Estimate")
row_name = "max_error_norm"
print("{}% error of standard".format(mything_df[row_name].mean() / standard_df[row_name].mean() * 100))
plt.plot(range(len(mything_df[row_name])), mything_df[row_name], 'g', label="PF + Short-term features")
plt.plot(range(len(mything_df[row_name])), [mything_df[row_name].mean()] * len(mything_df[row_name]), 'g')
plt.plot(range(len(standard_df[row_name])), standard_df[row_name], 'r', label="Standard PF")
plt.plot(range(len(standard_df[row_name])), [standard_df[row_name].mean()] * len(standard_df[row_name]), 'r')
plt.xlabel("Step")
plt.ylabel("Translational error (meters)")
plt.legend()
plt.grid()
plt.show()