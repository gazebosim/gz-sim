#!/usr/bin/env bash

N_X=20
N_Y=3
N_V=12
S=12345678

mkdir -p ${N_X}_${N_Y}_${N_V}
cd ${N_X}_${N_Y}_${N_V}

erb s=${S} n_x=${N_X} n_y=${N_Y} n_v=${N_V} ../dist_benchmark_primary.sdf.erb > distsim_p.sdf
erb s=${S} n_x=${N_X} n_y=${N_Y} n_v=${N_V} ../dist_benchmark_secondary.sdf.erb > distsim_s.sdf

cd ..
