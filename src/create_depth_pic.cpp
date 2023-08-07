#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/filesystem.hpp>
#include <ceres/ceres.h>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>
#include <pcl/point_types.h>
#include <string>
using namespace std;
string image_path = "../data/2.png";
string pointcloud_path = "../data/2.pcd";
// 针孔的
// 定义相机内参和畸变参数
cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 7.4491759362310563e+02, 0., 4.8292526824587895e+02, 
                                                  0., 7.4507701613189465e+02, 7.5934670470285107e+02, 
                                                  0., 0., 1. );
cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -3.5293886247001022e-01, 1.4904120245399277e-01,
                                                 1.6756729596747931e-04, -1.0791762368651489e-03,
                                                  -3.2626067564149284e-02);
// 外参（从雷达系到相机系）
    // double rx, ry, rz, x, y ,z;
    // cv::Mat rvec = (cv::Mat_<double>(3, 1) << rx, ry, rz);
    // cv::Mat tvec = (cv::Mat_<double>(3, 1) << x, y, z);

    // cv::Mat rvec = (cv::Mat_<double>(3, 1) << -0.02498880519919072, -0.1019245237703954, 0.007855506807299784);
    // cv::Mat tvec = (cv::Mat_<double>(3, 1) << -1.217420058762792, -0.5726543867687761, 2.569856609130538);
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0.012, 0.004, -0.011);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0.035, 0.1, -0.015);
    

    //0.012 0.004 -0.011 0.035 0.1 -0.015
// 定义点云类型

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

PointCloud::Ptr pointCloud(new PointCloud);  
cv::Mat image = cv::imread(image_path);
vector<cv::Mat> v1;
int main() {
    ////////////////////////
    cv::Mat rotatedImg;
    cv::rotate(image, rotatedImg, cv::ROTATE_90_COUNTERCLOCKWISE);
    /////////////////////////////////



    //读取点云文件
    pcl::io::loadPCDFile<pcl::PointXYZ>(pointcloud_path, *pointCloud);
    //点云到uv的转换
    double k1 = distCoeffs.at<double>(0,0);
    double k2 = distCoeffs.at<double>(0,1);
    double p1 = distCoeffs.at<double>(0,2);
    double p2 = distCoeffs.at<double>(0,3);
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cout<<"R is successful"<<endl;
    vector<cv::Vec3d> v1;
    // 遍历点云中的每个点
    for (const auto& point : pointCloud->points)
    {   
        
        // 将点的三维坐标转换为相机坐标系中的坐标
        cv::Mat point3D = (cv::Mat_<double>(3, 1) << point.x, point.y, point.z);
        cv::Mat point3D_cam = R * point3D + tvec;

        // 相机坐标到像素坐标
        double x = point3D_cam.at<double>(0,0)/point3D_cam.at<double>(2,0);
        double y = point3D_cam.at<double>(1,0)/point3D_cam.at<double>(2,0);
        double r2 = std::pow(x, 2) + std::pow(y, 2);
        double x1 = x * (1 + k1 * r2 + k2 * std::pow(r2, 2)) + 2 * p1 * y * x + p2 * (r2 + 2 * std::pow(x, 2));
        double y1 = y * (1 + k1 * r2 + k2 * std::pow(r2, 2)) + 2 * p2 * x * y + p1 * (r2 + 2 * std::pow(y, 2));
        double u = cameraMatrix.at<double>(0,0)* x1 + cameraMatrix.at<double>(0,2);
        double v = cameraMatrix.at<double>(1,1) * y1 + cameraMatrix.at<double>(1,2);
        // 根据深度设置颜色
        double distance = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2) + std::pow(point.z, 2));
        cv::Vec3d point_pix(u, v, distance);
        v1.push_back(point_pix);
        cv::circle(image, cv::Point(u, v), 1, cv::Scalar(0, 0, 255), cv::FILLED);
        cv::circle(rotatedImg, cv::Point(u, v), 1, cv::Scalar(0, 0, 255), cv::FILLED);

    }


    // 获取图像的宽度和高度
    int image_width = image.cols;
    int image_height = image.rows;

    // 创建深度图像，初始化为0
    cv::Mat depth_image(image_height, image_width, CV_64F, cv::Scalar(0));

    // 遍历v1中的每个像素点，将深度值填充到深度图像中
    for (const auto& point_pix : v1)
    {
         int u = static_cast<int>(point_pix[0]);
         int v = static_cast<int>(point_pix[1]);
         double distance = point_pix[2];


        // 将深度值填充到深度图像中，注意要进行边界检查
        if (u >= 0 && u < image_width && v >= 0 && v < image_height)
        {
            depth_image.at<double>(v, u) = distance;
        }
    }

    // 将深度图像归一化到0到255之间，方便可视化
    cv::Mat depth_image_norm;
    cv::normalize(depth_image, depth_image_norm, 155, 255, cv::NORM_MINMAX);

    //转换为无符号8位整型数据类型
    cv::Mat depth_image_8u;
    depth_image_norm.convertTo(depth_image_8u, CV_8U);

    //保存深度图像
    cv::imwrite("depth_image.png", depth_image_8u);

    // ///////////////////////////显示图像和深度图像
    cv::Mat rotatedImg2;
    cv::rotate(rotatedImg, rotatedImg2, cv::ROTATE_90_CLOCKWISE);
    cv::namedWindow("Projected Point Cloud", cv::WINDOW_NORMAL);
    cv::namedWindow("Depth_image", cv::WINDOW_NORMAL);
    cv::resizeWindow("Projected Point Cloud", 800, 600);
    cv::resizeWindow("Depth_image", 800, 600);
    cv::imshow("Projected Point Cloud", rotatedImg2);
    /////////////////

    //cv::imshow("original_image", image);
    cv::imshow("Depth_image", depth_image_8u);
    cv::waitKey(0);
    // cv::destroyAllWindows();

    
    return 0;
}
