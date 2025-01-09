/*
 * @Author: yijiu98 yijiu98@gmail.com
 * @Date: 2024-10-28 11:11:12
 * @LastEditors: yijiu98 yijiu98@gmail.com
 * @LastEditTime: 2024-11-13 14:53:09
 * @FilePath: /slambook/ch7/feature_extraction.cpp
 * @Description: 特征提取和匹配的实现代码
 */

#include <iostream> // 标准输入输出流
#include <opencv2/core/core.hpp> // OpenCV核心模块
#include <opencv2/features2d/features2d.hpp> // OpenCV特征提取模块
#include <opencv2/highgui/highgui.hpp> // OpenCV图像显示模块

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    // 检查输入参数是否正确
    if (argc != 3)
    {
        cout << "Usage: feature_extraction img1 img2" << endl; // 提示用法
        return 1; // 参数错误，返回非零值
    }

    //-- 读取图像
    Mat img_1 = imread(argv[1], IMREAD_COLOR); // 读取第一幅图像，彩色模式
    Mat img_2 = imread(argv[2], IMREAD_COLOR); // 读取第二幅图像，彩色模式

    if (img_1.empty() || img_2.empty())
    {
        // 检查图像是否成功加载
        cout << "Could not open or find the images!" << endl; // 如果图像加载失败，提示错误信息
        return 1; // 返回非零值
    }

    //-- 初始化
    vector<KeyPoint> keypoints_1, keypoints_2; // 用于存储图像的关键点
    Mat descriptors_1, descriptors_2; // 用于存储描述子
    Ptr<FeatureDetector> detector = ORB::create(); // 创建 ORB 特征检测器
    Ptr<DescriptorExtractor> descriptor = ORB::create(); // 创建 ORB 描述子提取器
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming"); // 创建描述子匹配器

    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(img_1, keypoints_1); // 检测第一幅图像的关键点
    detector->detect(img_2, keypoints_2); // 检测第二幅图像的关键点

    if (keypoints_1.empty() || keypoints_2.empty())
    {
        // 检查关键点是否检测成功
        cout << "No keypoints detected in one or both images!" << endl; // 如果没有检测到关键点，提示错误信息
        return 1; // 返回非零值
    }

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1); // 计算第一幅图像的描述子
    descriptor->compute(img_2, keypoints_2, descriptors_2); // 计算第二幅图像的描述子

    Mat outimg1;
    drawKeypoints(img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT); // 在图像上绘制关键点
    imshow("ORB Features", outimg1); // 显示带有关键点的第一幅图像

    //-- 第三步:对两幅图像中的 BRIEF 描述子进行匹配，使用 Hamming 距离
    vector<DMatch> matches; // 用于存储匹配结果
    matcher->match(descriptors_1, descriptors_2, matches); // 进行描述子匹配

    //-- 第四步:匹配点对筛选
    double min_dist = 10000, max_dist = 0; // 初始化最小和最大距离

    // 找出所有匹配点对之间的最小距离和最大距离
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        double dist = matches[i].distance; // 获取匹配点对的距离
        if (dist < min_dist) min_dist = dist; // 更新最小距离
        if (dist > max_dist) max_dist = dist; // 更新最大距离
    }

    // 或者使用标准库函数 min_element 和 max_element 进行距离筛选
    min_dist = min_element(matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {
        return m1.distance < m2.distance; // 比较两个匹配点对的距离
    })->distance;

    max_dist = max_element(matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {
        return m1.distance < m2.distance; // 比较两个匹配点对的距离
    })->distance;

    printf("-- Max dist : %f \n", max_dist); // 输出最大距离
    printf("-- Min dist : %f \n", min_dist); // 输出最小距离

    // 筛选出“好”的匹配点对
    vector<DMatch> good_matches; // 用于存储好的匹配点对
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (matches[i].distance <= max(2 * min_dist, 30.0)) // 距离小于 2 倍最小距离或 30.0
        {
            good_matches.push_back(matches[i]); // 将匹配点对添加到 good_matches 中
        }
    }

    //-- 第五步:绘制匹配结果
    Mat img_match; // 用于存储所有匹配的结果图像
    Mat img_goodmatch; // 用于存储优化后匹配的结果图像
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match); // 绘制所有匹配点对
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch); // 绘制优化后的匹配点对
    imshow("All Matches", img_match); // 显示所有匹配点对
    imshow("Optimized Matches", img_goodmatch); // 显示优化后的匹配点对
    waitKey(0); // 等待用户按键关闭窗口

    return 0; // 程序执行成功，返回 0
}
