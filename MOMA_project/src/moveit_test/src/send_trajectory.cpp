
#include <memory>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;


class JointTrajectoryClient : public rclcpp::Node
{
public:
  JointTrajectoryClient()
  : Node("joint_trajectory_client")
  {
    client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/tmr_arm_controller/follow_joint_trajectory");
  }

  void send_goal()
  {
    RCLCPP_INFO(get_logger(), "Waiting for action server...");
    bool server_available = client_->wait_for_action_server(5s);
    RCLCPP_INFO(get_logger(), "wait_for_action_server returned: %s", server_available ? "true" : "false");

    if (!server_available) {
      RCLCPP_ERROR(get_logger(), "Action server not available");
      rclcpp::shutdown();
      return;
    }

    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "Action server not available");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = {
      "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
    };

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.0, -1.57, 1.57, 0.0, 0.0, 0.0};
    point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Add velocities
    point.time_from_start = rclcpp::Duration::from_seconds(2.0);

    goal_msg.trajectory.points.push_back(point);

    RCLCPP_INFO(get_logger(), "Sending trajectory goal..."); // Gets stuck here

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.result_callback = [this](const GoalHandle::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "Trajectory execution succeeded.");
      } else {
        RCLCPP_WARN(get_logger(), "Trajectory failed with status: %d", static_cast<int>(result.code));
      }
      rclcpp::shutdown();
    };

    client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointTrajectoryClient>();

  // Send goal after node creation
  node->send_goal();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
