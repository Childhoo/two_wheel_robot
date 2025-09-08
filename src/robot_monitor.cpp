#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <iomanip>
#include <iostream>

class RobotMonitor : public rclcpp::Node
{
public:
    RobotMonitor() : Node("robot_monitor")
    {
        // 声明使用仿真时间
        this->declare_parameter("use_sim_time", true);
        
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, 
            std::bind(&RobotMonitor::cmd_vel_callback, this, std::placeholders::_1));
            
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&RobotMonitor::odom_callback, this, std::placeholders::_1));
            
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "robot_camera/image_raw", 10,
            std::bind(&RobotMonitor::image_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&RobotMonitor::display_status, this));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "Robot Monitor started");
        
        current_linear_vel_ = 0.0;
        current_angular_vel_ = 0.0;
        position_x_ = 0.0;
        position_y_ = 0.0;
        orientation_z_ = 0.0;
        camera_frame_count_ = 0;
        
        // 初始化时间为0，避免时间源问题
        last_cmd_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        last_odom_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        last_camera_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        current_linear_vel_ = msg->linear.x;
        current_angular_vel_ = msg->angular.z;
        last_cmd_time_ = this->get_clock()->now();
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        position_x_ = msg->pose.pose.position.x;
        position_y_ = msg->pose.pose.position.y;
        
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        orientation_z_ = yaw;
        
        last_odom_time_ = this->get_clock()->now();
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        camera_frame_count_++;
        last_camera_time_ = this->get_clock()->now();
        image_width_ = msg->width;
        image_height_ = msg->height;
        image_encoding_ = msg->encoding;
    }

    void display_status()
    {
        std::cout << "\033[2J\033[1;1H";
        std::cout << "====================================\n";
        std::cout << "      ROBOT STATUS MONITOR\n";
        std::cout << "====================================\n\n";
        
        std::cout << "📍 MOTION STATUS:\n";
        std::cout << "  Linear Velocity:  " << std::fixed << std::setprecision(3) 
                  << current_linear_vel_ << " m/s\n";
        std::cout << "  Angular Velocity: " << std::fixed << std::setprecision(3) 
                  << current_angular_vel_ << " rad/s\n";
        
        auto now = this->get_clock()->now();
        
        // 安全的时间计算
        double time_since_cmd = 999.0; // 默认很大的值
        if (last_cmd_time_.nanoseconds() > 0) {
            try {
                time_since_cmd = (now - last_cmd_time_).seconds();
            } catch (const std::exception& e) {
                time_since_cmd = 999.0;
            }
        }
        
        if (time_since_cmd > 2.0) {
            std::cout << "  Status: 🛑 STOPPED\n";
        } else if (abs(current_linear_vel_) > 0.01 || abs(current_angular_vel_) > 0.01) {
            std::cout << "  Status: 🚗 MOVING\n";
        } else {
            std::cout << "  Status: ⏸️  IDLE\n";
        }
        
        std::cout << "\n📊 POSITION:\n";
        std::cout << "  X: " << std::fixed << std::setprecision(3) << position_x_ << " m\n";
        std::cout << "  Y: " << std::fixed << std::setprecision(3) << position_y_ << " m\n";
        std::cout << "  Yaw: " << std::fixed << std::setprecision(3) 
                  << orientation_z_ * 180.0 / M_PI << " degrees\n";
        
        std::cout << "\n📷 CAMERA:\n";
        std::cout << "  Frames: " << camera_frame_count_ << "\n";
        std::cout << "  Resolution: " << image_width_ << "x" << image_height_ << "\n";
        
        double time_since_camera = 999.0;
        if (last_camera_time_.nanoseconds() > 0) {
            try {
                time_since_camera = (now - last_camera_time_).seconds();
            } catch (const std::exception& e) {
                time_since_camera = 999.0;
            }
        }
        
        if (time_since_camera > 2.0) {
            std::cout << "  Status: ❌ NO CAMERA DATA\n";
        } else {
            std::cout << "  Status: ✅ CAMERA ACTIVE\n";
        }
        
        std::cout << "\n====================================\n";
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    double current_linear_vel_, current_angular_vel_;
    double position_x_, position_y_, orientation_z_;
    uint32_t camera_frame_count_, image_width_, image_height_;
    std::string image_encoding_;
    rclcpp::Time last_cmd_time_, last_odom_time_, last_camera_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotMonitor>());
    rclcpp::shutdown();
    return 0;
}