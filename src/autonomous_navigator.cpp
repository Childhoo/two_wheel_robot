#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <random>

class AutonomousNavigator : public rclcpp::Node
{
public:
    AutonomousNavigator() : Node("autonomous_navigator")
    {
        // Publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // Subscribers
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "robot_camera/image_raw", 10,
            std::bind(&AutonomousNavigator::image_callback, this, std::placeholders::_1));
            
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&AutonomousNavigator::odom_callback, this, std::placeholders::_1));

        // Timer for navigation control
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&AutonomousNavigator::control_loop, this));

        // Timer for changing navigation patterns
        pattern_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5000),
            std::bind(&AutonomousNavigator::change_pattern, this));

        // Initialize random number generator
        rng_.seed(std::chrono::steady_clock::now().time_since_epoch().count());
        
        // Initialize state
        current_pattern_ = NavigationPattern::FORWARD;
        pattern_start_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Autonomous Navigator started");
        RCLCPP_INFO(this->get_logger(), "Robot will move autonomously in different patterns");
    }

private:
    enum class NavigationPattern {
        FORWARD,
        CIRCULAR,
        FIGURE_EIGHT,
        RANDOM_WALK,
        SPIRAL
    };

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Simple obstacle detection based on image brightness
        // This is a simplified approach - in real applications, you'd use
        // proper computer vision techniques
        has_camera_data_ = true;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        // Convert quaternion to yaw
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
    }

    void control_loop()
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        auto now = this->now();
        double elapsed_time = (now - pattern_start_time_).seconds();

        switch (current_pattern_) {
            case NavigationPattern::FORWARD:
                execute_forward_pattern(twist_msg, elapsed_time);
                break;
            case NavigationPattern::CIRCULAR:
                execute_circular_pattern(twist_msg, elapsed_time);
                break;
            case NavigationPattern::FIGURE_EIGHT:
                execute_figure_eight_pattern(twist_msg, elapsed_time);
                break;
            case NavigationPattern::RANDOM_WALK:
                execute_random_walk_pattern(twist_msg, elapsed_time);
                break;
            case NavigationPattern::SPIRAL:
                execute_spiral_pattern(twist_msg, elapsed_time);
                break;
        }

        cmd_vel_pub_->publish(twist_msg);
    }

    void execute_forward_pattern(geometry_msgs::msg::Twist& twist, double elapsed_time)
    {
        twist.linear.x = 0.5;  // Move forward at 0.5 m/s
        twist.angular.z = 0.0;
    }

    void execute_circular_pattern(geometry_msgs::msg::Twist& twist, double elapsed_time)
    {
        twist.linear.x = 0.3;   // Move forward
        twist.angular.z = 0.5;  // Turn at constant rate
    }

    void execute_figure_eight_pattern(geometry_msgs::msg::Twist& twist, double elapsed_time)
    {
        double frequency = 0.2;  // Hz
        twist.linear.x = 0.3;
        twist.angular.z = 0.8 * sin(2.0 * M_PI * frequency * elapsed_time);
    }

    void execute_random_walk_pattern(geometry_msgs::msg::Twist& twist, double elapsed_time)
    {
        // Change direction randomly every 2 seconds
        if (static_cast<int>(elapsed_time * 2) != last_random_change_) {
            std::uniform_real_distribution<double> linear_dist(0.1, 0.5);
            std::uniform_real_distribution<double> angular_dist(-1.0, 1.0);
            
            random_linear_vel_ = linear_dist(rng_);
            random_angular_vel_ = angular_dist(rng_);
            last_random_change_ = static_cast<int>(elapsed_time * 2);
        }
        
        twist.linear.x = random_linear_vel_;
        twist.angular.z = random_angular_vel_;
    }

    void execute_spiral_pattern(geometry_msgs::msg::Twist& twist, double elapsed_time)
    {
        twist.linear.x = 0.2 + elapsed_time * 0.05;  // Increasing linear speed
        twist.angular.z = 0.3;  // Constant angular velocity
    }

    void change_pattern()
    {
        // Randomly select a new navigation pattern
        std::uniform_int_distribution<int> pattern_dist(0, 4);
        int new_pattern_int = pattern_dist(rng_);
        
        NavigationPattern new_pattern = static_cast<NavigationPattern>(new_pattern_int);
        
        if (new_pattern != current_pattern_) {
            current_pattern_ = new_pattern;
            pattern_start_time_ = this->now();
            
            std::string pattern_name;
            switch (current_pattern_) {
                case NavigationPattern::FORWARD:
                    pattern_name = "FORWARD MOVEMENT";
                    break;
                case NavigationPattern::CIRCULAR:
                    pattern_name = "CIRCULAR MOVEMENT";
                    break;
                case NavigationPattern::FIGURE_EIGHT:
                    pattern_name = "FIGURE-EIGHT PATTERN";
                    break;
                case NavigationPattern::RANDOM_WALK:
                    pattern_name = "RANDOM WALK";
                    break;
                case NavigationPattern::SPIRAL:
                    pattern_name = "SPIRAL PATTERN";
                    break;
            }
            
            RCLCPP_INFO(this->get_logger(), "Switching to navigation pattern: %s", pattern_name.c_str());
        }
    }

    // Publishers and subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr pattern_timer_;
    
    // State variables
    NavigationPattern current_pattern_;
    rclcpp::Time pattern_start_time_;
    bool has_camera_data_ = false;
    
    // Current robot state
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;
    
    // Random number generation
    std::mt19937 rng_;
    double random_linear_vel_ = 0.0;
    double random_angular_vel_ = 0.0;
    int last_random_change_ = -1;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutonomousNavigator>());
    rclcpp::shutdown();
    return 0;
}