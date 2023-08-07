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

// 根据深度设置颜色
std::vector<int> set_RGB_from_distance(double color)
{
std::vector<int> v1;
int r, g, b;
        if (color <= 1.0)
        {
            r = 255;
            g = static_cast<int>(color * 255.0);
            b = 0;
        }
        else if (color <= 2.0)
        {
            r = static_cast<int>((2.0 - color) * 255.0);
            g = 255;
            b = 0;
        }
        else if (color <= 3.0)
        {
            r = 0;
            g = 255;
            b = static_cast<int>((color - 2.0) * 255.0);
        }
        else if (color <= 4.0)
        {
            r = 0;
            g = static_cast<int>((4.0 - color) * 255.0);
            b = 255;
        }
        else if (color <= 5.0)
        {
            r = static_cast<int>((color - 4.0) * 255.0);
            g = 0;
            b = 255;
        }
        else
        {
            r = 255;
            g = 0;
            b = static_cast<int>((6.0 - color) * 255.0);
        }
v1.push_back(r);
v1.push_back(g);
v1.push_back(b);
return v1;
}



// 点云投影到图像
// 输入点云 图片
void projectPointCloudToImage(const PointCloud::Ptr& pointCloud, cv::Mat& image)
{
    
    // 旋转向量转换为旋转矩阵
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    // 内参
    double k1 = distCoeffs.at<double>(0,0);
    double k2 = distCoeffs.at<double>(0,1);
    double p1 = distCoeffs.at<double>(0,2);
    double p2 = distCoeffs.at<double>(0,3);

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
        double color = distance * 6.0 / 10.0;
        std::vector<int> rgb = set_RGB_from_distance(color);
        // 绘制投影点
        cv::circle(image, cv::Point(u, v), 1, cv::Scalar(rgb[2], rgb[1], rgb[0]), cv::FILLED);
    }
   
    cv::Mat rotatedImg2;
    cv::rotate(image, rotatedImg2, cv::ROTATE_90_CLOCKWISE);
    // 显示结果图像
    cv::namedWindow("Projected Point Cloud", cv::WINDOW_NORMAL);
    cv::resizeWindow("Projected Point Cloud", 800, 600);
    cv::imshow("Projected Point Cloud", rotatedImg2);
    cv::waitKey(0);
}

int main()
{
    // 读取点云数据
    PointCloud::Ptr pointCloud(new PointCloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>("../data/2.pcd", *pointCloud);

    // 读取图像 逆时针旋转90度
    cv::Mat image = cv::imread("../data/2.png");
    cv::Mat rotatedImg;
    cv::rotate(image, rotatedImg, cv::ROTATE_90_COUNTERCLOCKWISE);
    // 将点云投影到图像上
    projectPointCloudToImage(pointCloud, rotatedImg);

    return 0;
}
