# lidar_camera_auto_calibration
A method to automatically calibrate lidar and camera
## Quickly start with the following steps
1.git clone this project
```
mkdir lidar_camera_calibration
cd lidar_camera_calibration
git clone 
1.compile code
```shell
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
