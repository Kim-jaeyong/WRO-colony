#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class HazardPublisher : public rclcpp::Node
{
public:
  HazardPublisher()
  : Node("hazard_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>(
      "/hazard_event", 10);

    timer_ = this->create_wall_timer(
      2s,
      [this]() {
        publish_hazard();
      });

    RCLCPP_INFO(this->get_logger(), "Hazard publisher started");
  }

private:
  void publish_hazard()
  {
    std_msgs::msg::String message;

    message.data =
      R"({"type":"FIRE","x":5.0,"y":3.0,"severity":100,"blocked":true})";

    publisher_->publish(message);

    RCLCPP_INFO(
      this->get_logger(),
      "Published: %s",
      message.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HazardPublisher>());
  rclcpp::shutdown();

  return 0;
}
