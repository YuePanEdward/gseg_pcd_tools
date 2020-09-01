#!/bin/sh

#data path
input_pointcloud_file=./test_data/test_pointcloud_cut.pcd
grid_def_file=./test_data/test_grids_def.txt
output_pointcloud_folder=./test_data/test_pointcloud_grids

./bin/gseg_pcd2grid ${input_pointcloud_file} ${grid_def_file} ${output_pointcloud_folder} 