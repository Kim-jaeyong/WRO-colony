#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class SwarmGoalDispatcher : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigate =
    rclcpp_action::ClientGoalHandle<NavigateToPose>;

  SwarmGoalDispatcher()
  : Node("swarm_goal_dispatcher")
  {
    goals_["robot1"] = {47.6, 5.5};
    goals_["robot2"] = {48.5, 5.5};

    for (const auto & [robot_name, goal] : goals_) {
      clients_[robot_name] =
        rclcpp_action::create_client<NavigateToPose>(
        this,
        "/" + robot_name + "/navigate_to_pose"
        );

      completed_[robot_name] = false;
    }

    timer_ = create_wall_timer(
      1s,
      std::bind(&SwarmGoalDispatcher::start_mission, this)
    );
  }

private:
  void start_mission()
  {
    timer_->cancel();

    for (const auto & [robot_name, position] : goals_) {
      auto client = clients_.at(robot_name);

      RCLCPP_INFO(
        get_logger(),
        "%s: waiting for action server",
        robot_name.c_str()
      );

      if (!client->wait_for_action_server(10s)) {
        RCLCPP_ERROR(
          get_logger(),
          "%s: action server unavailable",
          robot_name.c_str()
        );

        completed_[robot_name] = true;
        continue;
      }

      send_goal(robot_name, position.first, position.second);
    }
  }

  void send_goal(
    const std::string & robot_name,
    double x,
    double y)
  {
    NavigateToPose::Goal goal;

    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();

    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.position.z = 0.0;

    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = 0.0;
    goal.pose.pose.orientation.w = 1.0;

    auto options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    options.goal_response_callback =
      [this, robot_name](
      const GoalHandleNavigate::SharedPtr & goal_handle)
      {
        if (!goal_handle) {
          RCLCPP_ERROR(
            get_logger(),
            "%s: goal rejected",
            robot_name.c_str()
          );

          completed_[robot_name] = true;
          check_mission_complete();
          return;
        }

        RCLCPP_INFO(
          get_logger(),
          "%s: goal accepted",
          robot_name.c_str()
        );
      };

    options.result_callback =
      [this, robot_name](
      const GoalHandleNavigate::WrappedResult & result)
      {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(
            get_logger(),
            "%s: SUCCEEDED",
            robot_name.c_str()
          );
        } else {
          RCLCPP_ERROR(
            get_logger(),
            "%s: FAILED",
            robot_name.c_str()
          );
        }

        completed_[robot_name] = true;
        check_mission_complete();
      };

    RCLCPP_INFO(
      get_logger(),
      "%s: sending goal (%.1f, %.1f)",
      robot_name.c_str(),
      x,
      y
    );

    clients_.at(robot_name)->async_send_goal(goal, options);
  }

  void check_mission_complete()
  {
    for (const auto & [robot_name, complete] : completed_) {
      if (!complete) {
        return;
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Two-robot mission finished."
    );
  }

  std::unordered_map<
    std::string,
    std::pair<double, double>> goals_;

  std::unordered_map<
    std::string,
    rclcpp_action::Client<NavigateToPose>::SharedPtr> clients_;

  std::unordered_map<std::string, bool> completed_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SwarmGoalDispatcher>());
  rclcpp::shutdown();

  return 0;
}
