#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

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

  struct Waypoint
  {
    double x;
    double y;
  };

  struct RobotMission
  {
    std::string name;
    std::vector<Waypoint> waypoints;
    std::size_t current_index{0};
    int retry_count{0};
    bool finished{false};

    rclcpp_action::Client<NavigateToPose>::SharedPtr client;
    GoalHandleNavigate::SharedPtr active_goal;
  };

  SwarmGoalDispatcher()
  : Node("swarm_goal_dispatcher")
  {
    /*
     * 1차 탐색 경로
     *
     * robot1과 robot2가 평행하게 전진하며
     * 서로 다른 구역을 담당하는 기본 구조다.
     *
     * 이후 실제 건물 구조에 맞게 좌표만 바꾸면 된다.
     */
    add_robot(
      "robot1",
      {
        {47.6, 5.5},
        {47.6, 6.5},
        {47.6, 7.5}
      });

    add_robot(
      "robot2",
      {
        {48.5, 5.5},
        {48.5, 6.5},
        {48.5, 7.5}
      });

    start_timer_ = create_wall_timer(
      1s,
      std::bind(
        &SwarmGoalDispatcher::start_mission,
        this));
  }

private:
  static constexpr int MAX_RETRIES = 1;

  void add_robot(
    const std::string & robot_name,
    const std::vector<Waypoint> & waypoints)
  {
    RobotMission mission;

    mission.name = robot_name;
    mission.waypoints = waypoints;

    mission.client =
      rclcpp_action::create_client<NavigateToPose>(
      this,
      "/" + robot_name + "/navigate_to_pose");

    missions_.emplace(
      robot_name,
      std::move(mission));
  }

  void start_mission()
  {
    start_timer_->cancel();

    RCLCPP_INFO(
      get_logger(),
      "================================");

    RCLCPP_INFO(
      get_logger(),
      "Starting multi-robot exploration");

    RCLCPP_INFO(
      get_logger(),
      "================================");

    for (auto & [robot_name, mission] : missions_) {
      RCLCPP_INFO(
        get_logger(),
        "%s: waiting for Nav2 action server",
        robot_name.c_str());

      if (!mission.client->wait_for_action_server(15s)) {
        RCLCPP_ERROR(
          get_logger(),
          "%s: Nav2 action server unavailable",
          robot_name.c_str());

        mission.finished = true;
        continue;
      }

      RCLCPP_INFO(
        get_logger(),
        "%s: Nav2 action server connected",
        robot_name.c_str());

      send_next_waypoint(robot_name);
    }

    check_all_finished();
  }

  void send_next_waypoint(
    const std::string & robot_name)
  {
    auto & mission = missions_.at(robot_name);

    if (mission.finished) {
      return;
    }

    if (mission.current_index >= mission.waypoints.size()) {
      mission.finished = true;

      RCLCPP_INFO(
        get_logger(),
        "%s: exploration route completed",
        robot_name.c_str());

      check_all_finished();
      return;
    }

    const auto & waypoint =
      mission.waypoints.at(mission.current_index);

    NavigateToPose::Goal goal;

    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();

    goal.pose.pose.position.x = waypoint.x;
    goal.pose.pose.position.y = waypoint.y;
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
        auto & mission = missions_.at(robot_name);

        if (!goal_handle) {
          RCLCPP_ERROR(
            get_logger(),
            "%s: waypoint goal rejected",
            robot_name.c_str());

          handle_waypoint_failure(robot_name);
          return;
        }

        mission.active_goal = goal_handle;

        RCLCPP_INFO(
          get_logger(),
          "%s: waypoint %zu accepted",
          robot_name.c_str(),
          mission.current_index + 1);
      };

    options.result_callback =
      [this, robot_name](
      const GoalHandleNavigate::WrappedResult & result)
      {
        auto & mission = missions_.at(robot_name);
        mission.active_goal.reset();

        if (result.code ==
          rclcpp_action::ResultCode::SUCCEEDED)
        {
          RCLCPP_INFO(
            get_logger(),
            "%s: waypoint %zu reached",
            robot_name.c_str(),
            mission.current_index + 1);

          mission.retry_count = 0;
          mission.current_index++;

          send_next_waypoint(robot_name);
          return;
        }

        if (result.code ==
          rclcpp_action::ResultCode::CANCELED)
        {
          RCLCPP_WARN(
            get_logger(),
            "%s: waypoint canceled",
            robot_name.c_str());
        } else {
          RCLCPP_ERROR(
            get_logger(),
            "%s: waypoint navigation failed",
            robot_name.c_str());
        }

        handle_waypoint_failure(robot_name);
      };

    RCLCPP_INFO(
      get_logger(),
      "%s: moving to waypoint %zu/%zu (%.2f, %.2f)",
      robot_name.c_str(),
      mission.current_index + 1,
      mission.waypoints.size(),
      waypoint.x,
      waypoint.y);

    mission.client->async_send_goal(
      goal,
      options);
  }

  void handle_waypoint_failure(
    const std::string & robot_name)
  {
    auto & mission = missions_.at(robot_name);

    if (mission.retry_count < MAX_RETRIES) {
      mission.retry_count++;

      RCLCPP_WARN(
        get_logger(),
        "%s: retrying waypoint %zu",
        robot_name.c_str(),
        mission.current_index + 1);

      send_next_waypoint(robot_name);
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "%s: skipping unreachable waypoint %zu",
      robot_name.c_str(),
      mission.current_index + 1);

    mission.retry_count = 0;
    mission.current_index++;

    send_next_waypoint(robot_name);
  }

  void check_all_finished()
  {
    for (const auto & [robot_name, mission] : missions_) {
      if (!mission.finished) {
        return;
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "================================");

    RCLCPP_INFO(
      get_logger(),
      "All robot exploration routes completed");

    RCLCPP_INFO(
      get_logger(),
      "================================");
  }

  std::map<std::string, RobotMission> missions_;
  rclcpp::TimerBase::SharedPtr start_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<SwarmGoalDispatcher>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
