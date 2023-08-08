# lidar_camera_auto_calibration
A method to automatically calibrate lidar and camera
## Quickly start with the following steps
1.git clone this project
```shell
mkdir lidar_camera_calibration
cd lidar_camera_calibration
git clone git@github.com:louiszengCN/lidar_camera_auto_calibration.git
```
2.compile code
```shell
cd lidar_camera_calibration
mkdir build
cd build
cmake ..
make
```

### 1.create depth picture
```shell
cd build
./depth_demo
```
this code is to make a depth pichture with the pcd data, by using the extrinsic params between lidar and camera
we can project pointcloud onto camera picture

### 2.manual adjustment
```shell
cd build
./gui_demo
```
this code creates a gui which could help us to adjust the pointcloud manually

### 3.project pointcloud
```shell
cd build
./project_pcd
```
this code is a simple example that could project pointcloud onto camera picture
### 4.auto calibration
1.create folder
```
data
  |--img
      |--1.png #camera pictures
  |--pcd
      |--1.pcd #pcd cutted
```
2.set params
```cpp
nums = 2; // 数据集的数量 data里有几张图 就填多少 data数据越多 标定效果越好 一般6张以上较好
    banzi_row = 6;
    banzi_col = 5;
    squareSize = 0.1;
    // 相机的内参和去畸变参数 需要根据自己的相机进行更换
    cameraMatrix = (cv::Mat_<double>(3, 3) << 7.4491759362310563e+02, 0., 4.8292526824587895e+02, 
                                                  0., 7.4507701613189465e+02, 7.5934670470285107e+02, 
                                                  0., 0., 1. );
    distCoeffs = (cv::Mat_<double>(1, 5) << -3.5293886247001022e-01, 1.4904120245399277e-01,
                                                 1.6756729596747931e-04, -1.0791762368651489e-03,
                                                  -3.2626067564149284e-02);
```
3.recompile
```shell
cd build
cmake ..
make
./auto_calib
```
4.check result
```shell
cd build
-you can check corner detect result in build folder
-you can also use ./project_pic to visualize result, don't forget to update 
extrinsic in project.cpp
```
## Use your own data
1. change cameraMatrix and distCoeffs to your own camera params
```cpp
cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 7.4491759362310563e+02, 0., 4.8292526824587895e+02, 
                                                  0., 7.4507701613189465e+02, 7.5934670470285107e+02, 
                                                  0., 0., 1. );
cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -3.5293886247001022e-01, 1.4904120245399277e-01,
                                                 1.6756729596747931e-04, -1.0791762368651489e-03,
                                                  -3.2626067564149284e-02);
```
2. change image_path and pointcloud_path to your own data path
```cpp
string image_path = "../data/2.png";
string pointcloud_path = "../data/2.pcd";
```
3. change extrinsic params to your own data
```cpp
cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0.012, 0.004, -0.011);
cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0.035, 0.1, -0.015);
```
