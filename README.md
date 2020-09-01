# gseg_pcd_tools

## point cloud processing tools:

____________________________________________________________________________________
![Built](https://img.shields.io/appveyor/ci/gruntjs/grunt.svg)
____________________________________________________________________________________

 - [x] gseg_pcdcut: Cut a point cloud defined by a projected bounding polygon

 - [x] gesg_pcd2grid: Split a point cloud into several grid point clouds according to a file defining the grids

### How to use:

1. Install dependent libraries

Compulsory:

- [PCL(>=1.7)](https://github.com/PointCloudLibrary/pcl)

Optional:

- For *.las data IO: [LibLas](https://github.com/libLAS/libLAS)

If you are on ubuntu 14.04/16.04/18.04, you can simply use the apt to install pcl 1.7, as follows:

```
echo "install [pcl] 1.7"
echo "eigen, boost, flann, vtk involved in pcl"
sudo apt-get install libpcl-dev pcl-tools libproj-dev
echo "install [pcl] done"
```

2. Compile
```
mkdir build
cd build
cmake ..
make -j4
```

3. Run
```
cd ..
# configure the data path in script/run.sh file 
# the example datas are inside the test_data folder
# run the function
sh script/run_xxx.sh
```
