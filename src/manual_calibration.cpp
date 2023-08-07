#include <QApplication>
#include <QWidget>
#include <QGridLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <iostream>
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
//路径
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

// cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 3.2594317734549929e+02, 0., 5.6069316504706353e+02, 0.,
//        3.2590068625468155e+02, 5.5081160305584262e+02, 0., 0., 1.  );
// cv::Mat distCoeffs = (cv::Mat_<double>(1, 4) << -1.6206044439504120e-02, -3.6862178590853586e-04,
//        -1.5275824846988657e-03, 1.2624166711283183e-04);


// 外参（从雷达系到相机系）
    //double rx, ry, rz, x, y ,z;
    //rx: 0.42 ry: -0.19 rz: 1.495 x: -0.005 y: -0.01 z: 0.01
    //double rx= 0.365, ry= -0.13, rz= 1.56, x= -0.035, y= -0.325, z= 0.13;
    double rx= 0.42, ry= -0.19, rz= 1.495, x= -0.005, y= -0.01, z= 0.01;

    cv::Mat rvec = (cv::Mat_<double>(3, 1) << rx, ry, rz);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << x, y, z);
    //0.012 0.004 -0.011 0.035 0.1 -0.015
    //rx: 0.365 ry: -0.13 rz: 1.56 x: -0.035 y: -0.325 z: 0.13
// 定义点云类型
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


PointCloud::Ptr pointCloud(new PointCloud);  
cv::Mat image = cv::imread(image_path);
//cv::Mat rotatedImg;  


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
void projectPointCloudToImage(const PointCloud::Ptr& pointCloud, cv::Mat& image, cv::Mat _rvec, cv::Mat _tvec);

void get_ro_tran(double _rx, double _ry, double _rz, double _x, double _y, double _z)
{

    //创建变换矩阵
    cv::Mat rot_vector = (cv::Mat_<double>(3, 1) << _rx, _ry, _rz);
    cv::Mat tra_vector = (cv::Mat_<double>(3, 1) << _x, _y, _z);
    //更新参数
    rx =_rx;
    ry =_ry;
    rz =_rz;
    x =_x;
    y =_y;
    z =_z;
    cout<<"_rx"<<_rx<<endl;
    //输出当前更新量
    cout<<"rx: "<<rx<<" ry: "<<ry<<" rz: "<<rz<<" x: "<<x<<" y: "<<y<<" z: "<<z<<endl;
    //更新全局变量
    rvec = rot_vector;
    tvec = tra_vector;
    projectPointCloudToImage(pointCloud, image, rvec, tvec);

}

// 点云投影到图像
// 输入点云 图片
void projectPointCloudToImage(const PointCloud::Ptr& pointCloud, cv::Mat& image, cv::Mat _rvec, cv::Mat _tvec)
{
   
    //绘图的时候需要重新创建图片，否则点云会一直在图上
    cv::Mat image_new = cv::imread(image_path);
    // 旋转向量转换为旋转矩阵
    cv::Mat R;
    cv::Rodrigues(_rvec, R);
    // 内参
    double k1 = distCoeffs.at<double>(0,0);
    double k2 = distCoeffs.at<double>(0,1);
    double p1 = distCoeffs.at<double>(0,2);
    double p2 = distCoeffs.at<double>(0,3);
    cv::Mat distorted_points;  // 存储带畸变的像素点坐标
    cv::Mat undistorted_points;  // 存储去除畸变后的像素点坐标
    //遍历点云中的每个点
    for (const auto& point : pointCloud->points)
    {
        // 将点的三维坐标转换为相机坐标系中的坐标
        cv::Mat point3D = (cv::Mat_<double>(3, 1) << point.x, point.y, point.z);
        cv::Mat point3D_cam = R * point3D + _tvec;

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
        
        cv::circle(image_new, cv::Point(u, v), 1, cv::Scalar(rgb[2], rgb[1], rgb[0]), cv::FILLED);
        
    }

    //    for (const auto& point : pointCloud->points)
    // {
    //     // 将点的三维坐标转换为相机坐标系中的坐标
    //     cv::Mat point3D = (cv::Mat_<double>(3, 1) << point.x, point.y, point.z);
    //     cv::Mat point3D_cam = R * point3D + _tvec;
    //     cv::Mat distorted_points;  // 存储带畸变的像素点坐标
    //     cv::Mat undistorted_points;  // 存储去除畸变后的像素点坐标
    //     // 相机坐标到像素坐标
    //     double x = point3D_cam.at<double>(0,0)/point3D_cam.at<double>(2,0);
    //     double y = point3D_cam.at<double>(1,0)/point3D_cam.at<double>(2,0);
    //     double r2 = std::pow(x, 2) + std::pow(y, 2);
    //     double x1 = x * (1 + k1 * r2 + k2 * std::pow(r2, 2)) + 2 * p1 * y * x + p2 * (r2 + 2 * std::pow(x, 2));
    //     double y1 = y * (1 + k1 * r2 + k2 * std::pow(r2, 2)) + 2 * p2 * x * y + p1 * (r2 + 2 * std::pow(y, 2));
    //     double u = cameraMatrix.at<double>(0,0)* x1 + cameraMatrix.at<double>(0,2);
    //     double v = cameraMatrix.at<double>(1,1) * y1 + cameraMatrix.at<double>(1,2);
    //     double u_distorted = cameraMatrix.at<double>(0, 0) * x1 + cameraMatrix.at<double>(0, 2);
    //     double v_distorted = cameraMatrix.at<double>(1, 1) * y1 + cameraMatrix.at<double>(1, 2);
        
    //     // 存储畸变坐标
    //     distorted_points.push_back(cv::Point2f(u_distorted, v_distorted));
    //     cv::fisheye::undistortPoints(distorted_points, undistorted_points, cameraMatrix, distCoeffs);

    //     // 根据深度设置颜色
    //     double distance = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2) + std::pow(point.z, 2));
    //     double color = distance * 6.0 / 10.0;
    //     std::vector<int> rgb = set_RGB_from_distance(color);
    //     // 绘制投影点
        
    //     cv::circle(image_new, cv::Point(u_distorted, v_distorted), 1, cv::Scalar(rgb[2], rgb[1], rgb[0]), cv::FILLED);
        
    // }


    cv::Mat rotatedImg2;
    cv::rotate(image_new, rotatedImg2, cv::ROTATE_90_CLOCKWISE);
    // 显示结果图像
    cv::namedWindow("Projected Point Cloud", cv::WINDOW_NORMAL);
    cv::resizeWindow("Projected Point Cloud", 800, 600);
    cv::imshow("Projected Point Cloud", rotatedImg2);
    cv::waitKey(0);
}
// 添加其他Qt头文件，以实现其他必要的功能


int main(int argc, char *argv[])
{
    
    pcl::io::loadPCDFile<pcl::PointXYZ>(pointcloud_path, *pointCloud);
    
    QApplication app(argc, argv);

    // 创建主窗口
    QWidget window;
    window.setWindowTitle("Point Cloud Projection");

    // 创建布局
    QGridLayout *layout = new QGridLayout;

    // 创建标签和DoubleSpinBoxes
    //rx
    QLabel *label_rx = new QLabel("rx:");
    QDoubleSpinBox *spinBox_rx = new QDoubleSpinBox;
    spinBox_rx->setRange(-4.0, 4.0);
    spinBox_rx->setSingleStep(0.005);
    spinBox_rx->setValue(rx);
    QObject::connect(spinBox_rx, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [&](double value) {
         get_ro_tran(value, ry, rz, x, y ,z);});

    //ry
    QLabel *label_ry = new QLabel("ry:");
    QDoubleSpinBox *spinBox_ry = new QDoubleSpinBox;
    spinBox_ry->setRange(-4.0, 4.0);
    spinBox_ry->setSingleStep(0.005);
    spinBox_ry->setValue(ry);
    QObject::connect(spinBox_ry, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [&](double value) {
         get_ro_tran(rx, value, rz, x, y ,z);});

    //rz
    QLabel *label_rz = new QLabel("rz:");
    QDoubleSpinBox *spinBox_rz = new QDoubleSpinBox;
    spinBox_rz->setRange(-4.0, 4.0);
    spinBox_rz->setSingleStep(0.005);
    spinBox_rz->setValue(rz);
    QObject::connect(spinBox_rz, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [&](double value) {
        get_ro_tran(rx, ry, value, x, y ,z);});

    //x
    QLabel *label_x = new QLabel("x:");
    QDoubleSpinBox *spinBox_x = new QDoubleSpinBox;
    spinBox_x->setRange(-4.0, 4.0);
    spinBox_x->setSingleStep(0.005);
    spinBox_x->setValue(x);
    QObject::connect(spinBox_x, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [&](double value) {
        get_ro_tran(rx, ry, rz, value, y ,z);});

    //y
    QLabel *label_y = new QLabel("y:");
    QDoubleSpinBox *spinBox_y = new QDoubleSpinBox;
    spinBox_y->setRange(-4.0, 4.0);
    spinBox_y->setSingleStep(0.005);
    spinBox_y->setValue(y);
    QObject::connect(spinBox_y, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [&](double value) {
        get_ro_tran(rx, ry, rz, x, value ,z);});

    //z
    QLabel *label_z = new QLabel("rz:");
    QDoubleSpinBox *spinBox_z = new QDoubleSpinBox;
    spinBox_z->setRange(-4.0, 4.0);
    spinBox_z->setSingleStep(0.005);
    spinBox_z->setValue(z);
    QObject::connect(spinBox_z, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [&](double value) {
        get_ro_tran(rx, ry, rz, x, y , value);});

    // 添加标签和DoubleSpinBoxes到布局
    layout->addWidget(label_rx, 0, 0);
    layout->addWidget(spinBox_rx, 0, 1);
    layout->addWidget(label_ry, 1, 0);
    layout->addWidget(spinBox_ry, 1, 1);
    layout->addWidget(label_rz, 2, 0);
    layout->addWidget(spinBox_rz, 2, 1);
    layout->addWidget(label_x, 3, 0);
    layout->addWidget(spinBox_x, 3, 1);
    layout->addWidget(label_y, 4, 0);
    layout->addWidget(spinBox_y, 4, 1);
    layout->addWidget(label_z, 5, 0);
    layout->addWidget(spinBox_z, 5, 1);


    // 设置布局到主窗口
    window.setLayout(layout);
    window.show();

    return app.exec();
}
