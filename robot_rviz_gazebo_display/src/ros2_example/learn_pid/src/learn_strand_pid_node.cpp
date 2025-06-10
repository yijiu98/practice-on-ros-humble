#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

// 定义PID结构体
typedef struct
{
    float kp, ki, kd;        // 三个系数：比例、积分和微分
    float error, lastError;  // 当前误差、上次误差
    float integral, maxIntegral;  // 积分、积分限幅
    float output, maxOutput; // 输出、输出限幅
} PID;

// 用于初始化PID参数的函数
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut)
{
    pid->kp = p;//设置比例系数
    pid->ki = i;//设置积分系数
    pid->kd = d;//设置微分系数
    pid->maxIntegral = maxI;//设置积分限幅
    pid->maxOutput = maxOut;//设置输出限幅
    pid->error = 0;
    pid->lastError = 0;
    pid->integral = 0;
    pid->output = 0;
}

// 进行一次PID计算
void PID_Calc(PID *pid, float reference, float feedback)
{
    pid->lastError = pid->error;
    pid->error = reference - feedback;

    // 计算微分项
    float dout = (pid->error - pid->lastError) * pid->kd;

    // 计算比例项
    float pout = pid->error * pid->kp;

    // 计算积分项
    pid->integral += pid->error * pid->ki;

    // 积分限幅
    if (pid->integral > pid->maxIntegral)
        pid->integral = pid->maxIntegral;
    else if (pid->integral < -pid->maxIntegral)
        pid->integral = -pid->maxIntegral;

    // 计算输出
    pid->output = pout + dout + pid->integral;

    // 输出限幅
    if (pid->output > pid->maxOutput)
        pid->output = pid->maxOutput;
    else if (pid->output < -pid->maxOutput)
        pid->output = -pid->maxOutput;
}
//串级pid的机构提，包含两个串级pid
typedef struct
{
    PID inner;//内环
    PID outer;//外环
    float output;//串级输出，等于inner,output
}CascadePID;

//串级pid的计算函数
//参数pid（pid结构体，外环目标值，内环反馈值）
void PID_CascadeCalc(CascadePID* pid,float outerRef,float outerFdb,float innerFdb)
{
    PID_Calc(&pid->outer,outerRef,outerFdb);//计算外环
    PID_Calc(&pid->inner,pid->outer.output,innerFdb);//计算内环
    pid->output = pid->inner.output;//内环输出就是串级PID的输出
}
class PIDPublisherNode : public rclcpp::Node
{
public:
    PIDPublisherNode()
        : Node("pid_publisher_node"), feedback_value_(0.0)
    {
        // 初始化PID参数：比例系数2，积分系数0.1，微分系数1，最大积分500，最大输出1000
        // PID_Init(&mypid_, 2, 0.1, 1, 500, 1000);
        PID_Init(&mypid_.inner,10,0,0,0,1000);
        PID_Init(&mypid_.outer,5,0,5,0,100);
        pub_feedback_ = this->create_publisher<std_msgs::msg::Float64>("feedback",10);
        // 创建一个发布器，发布话题名为 /pid_output
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/pid_output", 10);

        // 创建一个定时器，定时调用PID计算并发布结果
        timer_ = this->create_wall_timer(
            100ms, std::bind(&PIDPublisherNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // 获取目标值和反馈值
        float outerTarget = getTargetValue();
        float outerFeedback = getFeedbackValue();
        float innerFeedback = getFeedbackValue();
        auto msg = std_msgs::msg::Float64();
        msg.data = outerFeedback;
        pub_feedback_->publish(msg);
        // 进行PID计算
        // PID_Calc(&mypid_, target_value, feedback_value);
        PID_CascadeCalc(&mypid_,outerTarget,outerFeedback,innerFeedback);

        // 打印输出值
        // RCLCPP_INFO(this->get_logger(), "Target: %.2f, Feedback: %.2f, PID Output: %.2f",
        //             target_value, feedback_value, mypid_.output);

        // 发布PID输出
        auto message = std_msgs::msg::Float64();
        message.data = mypid_.output;
        publisher_->publish(message);
    }

    float getTargetValue()
    {
        // 模拟获取目标值
        return 100.0; // 固定目标值
    }

    // float getFeedbackValue()
    // {
    //     // 模拟获取反馈值
    //     feedback_value_ += 1.0; // 模拟反馈值增加
    //     return feedback_value_;
    // }
    float getFeedbackValue()
    {
        // 模拟惯性响应：每次反馈值变化逐渐趋向目标值
        float rate_of_change = 0.1;  // 响应速率
        float target_value = 100.0;
        feedback_value_ += rate_of_change * (target_value - feedback_value_);  // 趋向目标值
        return feedback_value_;
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_; // 发布器
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_feedback_;
    rclcpp::TimerBase::SharedPtr timer_; // 定时器
    // PID mypid_; // PID结构体
    CascadePID mypid_;
    float feedback_value_; // 模拟反馈值
};

int main(int argc, char *argv[])
{
    // 初始化ROS 2节点
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
