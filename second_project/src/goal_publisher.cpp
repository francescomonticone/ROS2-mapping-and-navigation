#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class GoalPublisher : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  struct GoalData
  {
    double x;
    double y;
    double theta;
  };

  GoalPublisher()
  : Node("goal_publisher"), current_goal_idx_(0), waiting_for_result_(false)
  {
    this->declare_parameter<std::string>("csv_path", "");
    this->declare_parameter<std::string>("action_name", "navigate_to_pose");
    this->declare_parameter<std::string>("frame_id", "map");
    if (!this->has_parameter("use_sim_time")) {
    this->declare_parameter("use_sim_time", true);
    }
    if (!this->has_parameter("csv_file")) {
        this->declare_parameter("csv_file", std::string(""));
    }

    csv_path_ = this->get_parameter("csv_path").as_string();
    action_name_ = this->get_parameter("action_name").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();

    if (csv_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'csv_path' is empty.");
      rclcpp::shutdown();
      return;
    }

    if (!load_goals_from_csv(csv_path_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load goals from CSV: %s", csv_path_.c_str());
      rclcpp::shutdown();
      return;
    }

    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);

    startup_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&GoalPublisher::try_start, this));

    RCLCPP_INFO(this->get_logger(), "Goal publisher ready. Loaded %zu goals from %s",
      goals_.size(), csv_path_.c_str());

    
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr startup_timer_;

  std::vector<GoalData> goals_;
  std::size_t current_goal_idx_;
  bool waiting_for_result_;

  std::string csv_path_;
  std::string action_name_;
  std::string frame_id_;

  bool load_goals_from_csv(const std::string & path)
  {
    std::ifstream file(path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open CSV file: %s", path.c_str());
      return false;
    }

    std::string line;
    std::size_t line_number = 0;

    while (std::getline(file, line)) {
      ++line_number;

      if (line.empty()) {
        continue;
      }

      if (line_number == 1 && line.find("x") != std::string::npos) {
        continue;
      }

      std::stringstream ss(line);
      std::string sx, sy, stheta;

      if (!std::getline(ss, sx, ',')) {
        RCLCPP_WARN(this->get_logger(), "Skipping malformed line %zu", line_number);
        continue;
      }
      if (!std::getline(ss, sy, ',')) {
        RCLCPP_WARN(this->get_logger(), "Skipping malformed line %zu", line_number);
        continue;
      }
      if (!std::getline(ss, stheta, ',')) {
        RCLCPP_WARN(this->get_logger(), "Skipping malformed line %zu", line_number);
        continue;
      }

      try {
        GoalData goal;
        goal.x = std::stod(sx);
        goal.y = std::stod(sy);
        goal.theta = std::stod(stheta);
        goals_.push_back(goal);
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(),
          "Skipping line %zu due to conversion error: %s", line_number, e.what());
      }
    }

    return !goals_.empty();
  }

  void try_start()
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Waiting for action server '%s'...", action_name_.c_str());
      return;
    }

    startup_timer_->cancel();
    send_next_goal();
  }

  void send_next_goal()
  {
    if (waiting_for_result_) {
      return;
    }

    if (current_goal_idx_ >= goals_.size()) {
      RCLCPP_INFO(this->get_logger(), "All goals processed.");
      rclcpp::shutdown();
      return;
    }

    const auto & g = goals_[current_goal_idx_];

    NavigateToPose::Goal goal_msg;
    goal_msg.pose.header.frame_id = frame_id_;
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = g.x;
    goal_msg.pose.pose.position.y = g.y;
    goal_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, g.theta);
    q.normalize();
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    RCLCPP_INFO(this->get_logger(),
      "Sending goal %zu/%zu -> x=%.3f y=%.3f theta=%.3f",
      current_goal_idx_ + 1, goals_.size(), g.x, g.y, g.theta);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options;
    send_goal_options.goal_response_callback =
      std::bind(&GoalPublisher::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&GoalPublisher::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&GoalPublisher::result_callback, this, std::placeholders::_1);

    waiting_for_result_ = true;
    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal %zu rejected by server.", current_goal_idx_ + 1);
      waiting_for_result_ = false;
      ++current_goal_idx_;
      send_next_goal();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Goal %zu accepted by server.", current_goal_idx_ + 1);
  }

  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Goal %zu feedback: distance remaining = %.3f",
      current_goal_idx_ + 1, feedback->distance_remaining);
  }

  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Goal %zu succeeded.", current_goal_idx_ + 1);
      waiting_for_result_ = false;
      ++current_goal_idx_;
      send_next_goal();
      return;
    }

    if (result.code == rclcpp_action::ResultCode::ABORTED) {
      RCLCPP_WARN(this->get_logger(), "Goal %zu aborted. Moving to next goal.", current_goal_idx_ + 1);
      waiting_for_result_ = false;
      ++current_goal_idx_;
      send_next_goal();
      return;
    }

    if (result.code == rclcpp_action::ResultCode::CANCELED) {
      RCLCPP_WARN(this->get_logger(), "Goal %zu canceled. Stopping sequence.", current_goal_idx_ + 1);
      rclcpp::shutdown();
      return;
    }

    RCLCPP_ERROR(this->get_logger(), "Goal %zu returned unknown result. Stopping sequence.", current_goal_idx_ + 1);
    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}