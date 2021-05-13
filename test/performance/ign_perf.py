#!/usr/bin/env python3

import argparse
import numpy as np
import csv

import matplotlib.pyplot as plt

def read_data(filename):
    header = []
    entries = []

    with open(filename) as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='"')
        for row in reader:
            if row[0][0] == '#':
                header.append(row)
            else:
                entries.append([float(r) for r in row])
    return (header, np.array(entries))

def compute_rtfs(real_time, sim_time):
    # Compute time deltas
    real_dt = np.diff(real_time)
    sim_dt = np.diff(sim_time)
    # Compute rtf and some statistics
    rtfs = sim_dt / real_dt
    return rtfs

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('filename')
    parser.add_argument('--summarize', action='store_true')
    parser.add_argument('--plot', action='store_true')
    parser.add_argument('--hist', action='store_true')
    args = parser.parse_args()

    (header, data) = read_data(args.filename)

    real_time = data[:,0] + 1e-9 * data[:,1]
    sim_time = data[:,2] + 1e-9 * data[:,3]

    rtfs = compute_rtfs(real_time, sim_time)

    if args.summarize:
        iters = len(data)

        mean = np.mean(rtfs)
        median = np.median(rtfs)
        mn = np.min(rtfs)
        mx = np.max(rtfs)

        amn = np.argmin(rtfs)
        amx = np.argmax(rtfs)
        mn_sim = sim_time[amn]
        mx_sim = sim_time[amx]
        mn_real = real_time[amn]
        mx_real = real_time[amx]

        print(f'Iterations: {iters}')
        print(f'Mean RTF:   {mean:0.5f}')
        print(f'Median RTF: {median:0.5f}')
        print(f'Min RTF:    {mn:0.5f}')
        print(f'  Iteration: {amn}')
        print(f'  Sim Time:  {mn_sim}')
        print(f'  Real Time: {mn_real:0.5f}')
        print(f'Max RTF:    {mx:0.5f}')
        print(f'  Iteration: {amx}')
        print(f'  Sim Time:  {mx_sim:0.5f}')
        print(f'  Real Time: {mx_real:0.5f}')

    if args.plot:
        plt.figure()
        plt.plot(sim_time[:-1], rtfs)
        plt.title('Sim Time vs Real Time Factor')
        plt.xlabel('Sim Time (s)')
        plt.ylabel('Real Time Factor')
        plt.grid(True)

    if args.hist:
        plt.figure()
        _ = plt.hist(rtfs, bins=100)
        plt.title('Real Time Factor Histogram')
        plt.xlabel('Real Time Factor')
        plt.ylabel('Iteration Count')
        plt.grid(True)


    plt.show()
