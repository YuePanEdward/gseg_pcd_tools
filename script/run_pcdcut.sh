#!/bin/sh

#data path
input_pointcloud_file=./test_data/test_pointcloud.pcd
bounding_polygon_file=./test_data/test_polygon.txt
plane_coefficients_file=./test_data/test_plane_coefficients.txt
output_pointcloud_file=./test_data/test_pointcloud_cut.pcd

./bin/gseg_pcdcut ${input_pointcloud_file} ${bounding_polygon_file} ${output_pointcloud_file} ${plane_coefficients_file}