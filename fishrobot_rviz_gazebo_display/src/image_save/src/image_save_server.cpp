#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <chrono>
#include <fstream>
#include "image_save/srv/save_depth_data.hpp"

namespace fs = std::filesystem;

class DepthCameraService : public rclcpp::Node {
public:
    DepthCameraService()
        : Node("image_save_service"),
          tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
          tf_listener_(tf_buffer_) {
        service_ = this->create_service<image_save::srv::SaveDepthData>(
            "save_depth_data", std::bind(&DepthCameraService::handle_request, this, std::placeholders::_1, std::placeholders::_2));

        depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera_sensor/depth/image_raw", 10,
            std::bind(&DepthCameraService::depth_image_callback, this, std::placeholders::_1));

        color_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera_sensor/image_raw", 10,
            std::bind(&DepthCameraService::color_image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Depth Camera Service is ready.");
    }

private:
    void handle_request(const std::shared_ptr<image_save::srv::SaveDepthData::Request> request,
                        std::shared_ptr<image_save::srv::SaveDepthData::Response> response) {
        std::string save_path = request->save_path;
        if (depth_image_16u.empty() || color_image_.empty()) {
            response->success = false;
            response->message = "No image data received yet.";
            return;
        }

        try {
            // 获取当前时间戳
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
            std::ostringstream timestamp;
            timestamp << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << "_" << ms.count();

            // 确保保存目录存在
            fs::create_directories(save_path);

            // 保存深度图像为PGM
            std::string pgm_file = save_path + "/" + timestamp.str() + "_depth.pgm";
            cv::imwrite(pgm_file, depth_image_16u);

            // 保存彩色图像为PNG
            std::string png_file = save_path + "/" + timestamp.str() + "_color.png";
            cv::imwrite(png_file, color_image_);

            // 获取TF odom坐标
            geometry_msgs::msg::TransformStamped transform;
            try {
                transform = tf_buffer_.lookupTransform("odom", "base_link", tf2::TimePointZero);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform odom to base_link: %s", ex.what());
                response->success = false;
                response->message = "TF lookup failed.";
                return;
            }

            // 保存位姿到文件
            std::string pose_file = save_path + "/" + timestamp.str() + "_pose.txt";
            std::ofstream pose_output(pose_file);
            pose_output << "Position: [" << transform.transform.translation.x << ", "
                        << transform.transform.translation.y << ", "
                        << transform.transform.translation.z << "]\n";
            pose_output << "Orientation: [" << transform.transform.rotation.x << ", "
                        << transform.transform.rotation.y << ", "
                        << transform.transform.rotation.z << ", "
                        << transform.transform.rotation.w << "]\n";
            pose_output.close();

            response->success = true;
            response->message = "Files saved successfully.";
            RCLCPP_INFO(this->get_logger(), "Files saved to %s", save_path.c_str());
        } catch (const std::exception &e) {
            response->success = false;
            response->message = e.what();
            RCLCPP_ERROR(this->get_logger(), "Failed to save files: %s", e.what());
        }
    }

    void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            if (cv_ptr->image.type() != CV_32F)
            {
                RCLCPP_ERROR(this->get_logger(), "Image is not in 32-bit single-channel format! is %d",cv_ptr->image.type() );
            }
             // 获取原始深度图像
            depth_image_ = cv_ptr->image;

            // 将 CV_32F 转换为 CV_16U
            
            double min_val, max_val;
            cv::minMaxLoc(depth_image_, &min_val, &max_val); // 获取深度图的最大值和最小值
            double scale = 65535.0 / max_val; // 映射到 16 位范围

            depth_image_.convertTo(depth_image_16u, CV_16U, scale);
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
        }
    }

    void color_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            color_image_ = cv_ptr->image;
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
        }
    }

    rclcpp::Service<image_save::srv::SaveDepthData>::SharedPtr service_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
    cv::Mat depth_image_;
    cv::Mat color_image_;
    cv::Mat depth_image_16u;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthCameraService>());
    rclcpp::shutdown();
    return 0;
}
