#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class TwistPublisher : public rclcpp::Node
{
public:
  TwistPublisher()
  : Node("twist_publisher")
  {
    // Create a publisher for the /diff_drive/cmd_vel topic
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diff_drive/cmd_vel", 10);

    // Create a timer to publish messages at 10 Hz (every 100ms)
    timer_ = this->create_wall_timer(
      100ms, std::bind(&TwistPublisher::publish_twist, this));
  }

private:
  void publish_twist()
  {
    // Create a Twist message
    auto message = geometry_msgs::msg::Twist();
    
    // Set linear and angular velocities (example values)
    message.linear.x = 0.5;  // Move forward at 0.5 m/s
    message.linear.y = 0.0;
    message.linear.z = 0.0;
    message.angular.x = 0.0;
    message.angular.y = 0.0;
    message.angular.z = 0.2; // Rotate at 0.2 rad/s

    // Publish the message
    RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%.2f, angular.z=%.2f",
                message.linear.x, message.angular.z);
    publisher_->publish(message);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistPublisher>());
  rclcpp::shutdown();
  return 0;
}
