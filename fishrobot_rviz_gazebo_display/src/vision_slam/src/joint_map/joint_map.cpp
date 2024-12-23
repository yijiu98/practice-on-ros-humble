#include <iostream>
#include <fstream>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry> 
#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 

int main( int argc, char** argv )
{
    // 定义用于存储彩色图像和深度图像的容器
    vector<cv::Mat> colorImgs, depthImgs;    
    // 定义存储相机位姿的容器，使用Eigen的Isometry3d表示位姿
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses; //前面是变换矩阵，后面的是分配器        

    // 打开包含相机位姿的文件 pose.txt
    ifstream fin("./pose.txt");
    if (!fin) // 如果文件无法打开
    {
        cerr<<"请在有pose.txt的目录下运行此程序"<<endl; // 输出错误提示
        return 1; // 程序退出
    }
    
    // 读取图像和相机位姿数据
    for ( int i=0; i<5; i++ ) // 假设有5组数据
    {
        boost::format fmt( "./%s/%d.%s" ); // 定义图像文件的格式化路径
        colorImgs.push_back( cv::imread( (fmt%"color"%(i+1)%"png").str() )); // 读取彩色图像
        depthImgs.push_back( cv::imread( (fmt%"depth"%(i+1)%"pgm").str(), -1 )); // 读取深度图像，使用-1保持原始数据格式
        
        double data[7] = {0}; // 用于存储位姿数据（3个平移+四元数）
        for ( auto& d:data ) // 逐个读取位姿数据
            fin>>d; // 从文件中读取
        Eigen::Quaterniond q( data[6], data[3], data[4], data[5] ); // 构造四元数
        Eigen::Isometry3d T(q); // 通过四元数构造变换矩阵（位姿）
        T.pretranslate( Eigen::Vector3d( data[0], data[1], data[2] )); // 添加平移向量
        poses.push_back( T ); // 将位姿存入容器
    }
    
    // 计算点云并拼接
    // 定义相机的内参
    double cx = 325.5; // 主点的x坐标
    double cy = 253.5; // 主点的y坐标
    double fx = 518.0; // 焦距在x方向的像素尺寸
    double fy = 519.0; // 焦距在y方向的像素尺寸
    double depthScale = 1000.0; // 深度缩放比例，将深度值从整数转换为米
    
    cout<<"正在将图像转换为点云..."<<endl; // 提示点云转换过程开始
    
    // 定义点云格式，这里使用 PCL 的 PointXYZRGB 表示 XYZ 和颜色信息
    typedef pcl::PointXYZRGB PointT; 
    typedef pcl::PointCloud<PointT> PointCloud; 
    
    // 创建一个空的点云指针
    PointCloud::Ptr pointCloud( new PointCloud ); 
    for ( int i=0; i<5; i++ ) // 遍历每一组图像和位姿数据
    {
        cout<<"转换图像中: "<<i+1<<endl; // 打印转换进度
        cv::Mat color = colorImgs[i]; // 获取彩色图像
        cv::Mat depth = depthImgs[i]; // 获取深度图像
        Eigen::Isometry3d T = poses[i]; // 获取对应的相机位姿
        
        // 遍历彩色图像的每一个像素点
        for ( int v=0; v<color.rows; v++ )
            for ( int u=0; u<color.cols; u++ )
            {
                unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 从深度图像中获取深度值
                if ( d==0 ) continue; // 跳过无效深度值
                Eigen::Vector3d point; // 定义相机坐标系下的3D点
                point[2] = double(d)/depthScale; // 计算z坐标（深度值转米）
                point[0] = (u-cx)*point[2]/fx; // 计算x坐标
                point[1] = (v-cy)*point[2]/fy; // 计算y坐标
                Eigen::Vector3d pointWorld = T*point; // 将点从相机坐标系变换到世界坐标系
                
                PointT p ; // 定义PCL点类型
                p.x = pointWorld[0]; // 设置点的x坐标
                p.y = pointWorld[1]; // 设置点的y坐标
                p.z = pointWorld[2]; // 设置点的z坐标
                // 从彩色图像中提取RGB颜色
                p.b = color.data[ v*color.step+u*color.channels() ]; // 蓝色通道
                p.g = color.data[ v*color.step+u*color.channels()+1 ]; // 绿色通道
                p.r = color.data[ v*color.step+u*color.channels()+2 ]; // 红色通道
                pointCloud->points.push_back( p ); // 将点添加到点云中
            }
    }
    
    pointCloud->is_dense = false; // 设置点云为非稠密
    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl; // 打印点云中的点数
    pcl::io::savePCDFileBinary("map.pcd", *pointCloud ); // 将点云保存为二进制格式的PCD文件
    return 0; // 程序结束
}
