#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class LocalHazardPublisher : public rclcpp::Node
{
public:
  LocalHazardPublisher()
  : Node("local_hazard_publisher")
  {
    // 실행할 때 robot1, robot2처럼 로봇 이름을 설정한다.
    robot_id_ = this->declare_parameter<std::string>(
      "robot_id",
      "unknown_robot"
    );

    // 앞에 /가 없는 상대 토픽 이름이다.
    publisher_ = this->create_publisher<std_msgs::msg::String>(
      "hazard_event",
      10
    );

    // 테스트를 위해 2초마다 화재 정보를 발행한다.
    timer_ = this->create_wall_timer(
      2s,
      [this]() {
        publish_hazard();
      }
    );

    RCLCPP_INFO(
      this->get_logger(),
      "%s local hazard node started",
      robot_id_.c_str()
    );
  }

private:
  void publish_hazard()
  {
    std_msgs::msg::String message;

    message.data =
      "{\"robot_id\":\"" + robot_id_ +
      "\",\"type\":\"FIRE\""
      ",\"x\":5.0"
      ",\"y\":3.0"
      ",\"severity\":100"
      ",\"blocked\":true}";

    publisher_->publish(message);

    RCLCPP_INFO(
      this->get_logger(),
      "Published: %s",
      message.data.c_str()
    );
  }

  std::string robot_id_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<LocalHazardPublisher>()
  );

  rclcpp::shutdown();

  return 0;
}
