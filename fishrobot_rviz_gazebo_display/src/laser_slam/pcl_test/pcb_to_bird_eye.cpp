//
// Created by xiang on 2021/8/9.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>  // 图像编解码功能[1](@ref)
#include <opencv2/highgui.hpp>    // 显示功能[2](@ref)
#include <opencv2/core/types_c.h> // 兼容旧版数据类型（如CV_8UC3）

// using PointType = pcl::PointXYZI;
using PointType = pcl::PointXYZ;// 定义点云类型（使用无强度值的XYZ结构体[6](@ref)）// 包含x,y,z三个float成员
using PointCloudType = pcl::PointCloud<PointType>;// 点云容器定义

DEFINE_string(pcd_path, "../../material/pcdexample/map_example.pcd", "点云文件路径");
DEFINE_double(image_resolution, 0.1, "俯视图分辨率");
DEFINE_double(min_z, 0.2, "俯视图最低高度");
DEFINE_double(max_z, 2.5, "俯视图最高高度");

/// 生成鸟瞰图函数
void GenerateBEVImage(PointCloudType::Ptr cloud) {
    // 计算点云XYZ坐标范围[4](@ref)
    auto minmax_x = std::minmax_element(cloud->points.begin(), cloud->points.end(),
                                        [](const PointType& p1, const PointType& p2) { return p1.x < p2.x; });
    auto minmax_y = std::minmax_element(cloud->points.begin(), cloud->points.end(),
                                        [](const PointType& p1, const PointType& p2) { return p1.y < p2.y; });
    // 获取XY边界值
    double min_x = minmax_x.first->x;
    double max_x = minmax_x.second->x;
    double min_y = minmax_y.first->y;
    double max_y = minmax_y.second->y;

    // 计算图像尺寸（基于分辨率和坐标范围）
    const double inv_r = 1.0 / FLAGS_image_resolution; // 分辨率倒数用于坐标转换

    const int image_rows = int((max_y - min_y) * inv_r); // 图像高度（行数）
    const int image_cols = int((max_x - min_x) * inv_r); // 图像宽度（列数）
    // 计算坐标映射参数
    float x_center = 0.5 * (max_x + min_x); // 点云X中心坐标
    float y_center = 0.5 * (max_y + min_y); // 点云Y中心坐标
    float x_center_image = image_cols / 2;// 图像X中心像素坐标
    float y_center_image = image_rows / 2;// 图像Y中心像素坐标

    // 创建白色背景图像（BGR格式，3通道）[4](@ref)
    cv::Mat image(image_rows, image_cols, CV_8UC3, cv::Scalar(255, 255, 255));
    // 遍历所有点云数据
    for (const auto& pt : cloud->points) {
        // 将世界坐标系转换为图像像素坐标
        int x = int((pt.x - x_center) * inv_r + x_center_image);
        int y = int((pt.y - y_center) * inv_r + y_center_image);
        if(!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }
        // 过滤无效点（超出图像范围或高度不符合）
        if (x < 0 || x >= image_cols || y < 0 || y >= image_rows || pt.z < FLAGS_min_z || pt.z > FLAGS_max_z) {
            continue;
        }
        // 设置像素颜色（BGR格式：227,143,79对应橙色）
        image.at<cv::Vec3b>(y, x) = cv::Vec3b(227, 143, 79);
    }

    cv::imwrite("./bev.png", image);
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_pcd_path.empty()) {
        LOG(ERROR) << "pcd path is empty";
        return -1;
    }

    // 加载点云数据（显式指定PointXYZ类型[6](@ref)）
    PointCloudType::Ptr cloud(new PointCloudType);
    if(pcl::io::loadPCDFile(FLAGS_pcd_path, *cloud)== -1){
        LOG(ERROR)<<"Failed to load cloud file";
        return -1;
    }
    // 检查点云有效性
    if (cloud->empty()) {
        LOG(ERROR) << "cannot load cloud file";
        return -1;
    }

    LOG(INFO) << "Successfully loaded"<<"cloud points: " << cloud->size();
    GenerateBEVImage(cloud);

    return 0;
}
