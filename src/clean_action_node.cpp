// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <math.h>

#include <memory>
#include <string>
#include <map>
#include <algorithm>
#include <functional>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_msgs/action/navigate_through_poses.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_action_clients/action_observed_cost_client.hpp"

#include "plansys2_actions_cost/move_through_poses_action_cost_base.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include "tf2/LinearMath/Quaternion.h"
// #include "tf2/LinearMath/Transform.h"
// #include "tf2/LinearMath/Vector3.h"

#include <pluginlib/class_loader.hpp>

using namespace std::chrono_literals;

class CleanAction : public plansys2_actions_clients::ActionObservedCostClient
{
public:
  CleanAction()
  : plansys2_actions_clients::ActionObservedCostClient("clean", 500ms) //: move_action_cost_loader_("plansys2_actions_cost", "plansys2_actions_cost::MoveActionCostBase")
  {
    RCLCPP_INFO(get_logger(), "CleanAction created");

    tf_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    RCLCPP_INFO(get_logger(), "CleanAction created");
    declare_parameter<std::string>("namespace", "");
    namespace_ = get_parameter("namespace").as_string();
    
    declare_parameter<std::vector<std::string>>("start_wp", std::vector<std::string>());
    auto start_waypoints = get_parameter("start_wp").as_string_array();
    RCLCPP_INFO(get_logger(), "Cleaning node");
    for(const auto& start_wp_name : start_waypoints)
    {
      declare_parameter<std::vector<std::string>>(start_wp_name, std::vector<std::string>());
      std::vector<std::string> cleaning_waypoints_names = get_parameter(start_wp_name).as_string_array();
      RCLCPP_INFO(get_logger(), "Cleaning start waypoints [%s]", start_wp_name.c_str());
      cleaning_waypoints_[start_wp_name] = std::vector<geometry_msgs::msg::PoseStamped>();
      for(const auto& cleaning_waypoint: cleaning_waypoints_names)
      {
        std::string cleaning_waypoints_name = std::string(start_wp_name);
        declare_parameter<double>(cleaning_waypoint + ".position.x", 0.0);
        declare_parameter<double>(cleaning_waypoint + ".position.y", 0.0);
        declare_parameter<double>(cleaning_waypoint + ".orientation.x", 0.0);
        declare_parameter<double>(cleaning_waypoint + ".orientation.y", 0.0);
        declare_parameter<double>(cleaning_waypoint + ".orientation.z", 0.0);
        declare_parameter<double>(cleaning_waypoint + ".orientation.w", 1.0);

        // declare_parameter<double>(cleaning_waypoint + "position.z", 0.0);
        double x = get_parameter(cleaning_waypoint + ".position.x").as_double();
        double y = get_parameter(cleaning_waypoint + ".position.y").as_double();
        // double yaw = get_parameter(cleaning_waypoint + ".yaw").as_double();

        RCLCPP_INFO(get_logger(), "Cleaning waypoint [%s] x: %f, y: %f", cleaning_waypoint.c_str(), x, y);

        geometry_msgs::msg::PoseStamped wp;
        wp.header.frame_id = "map";
        wp.header.stamp = now();
        wp.pose.position.x = x;
        wp.pose.position.y = y;
        wp.pose.position.z = 0.0;
        wp.pose.orientation.x = get_parameter(cleaning_waypoint + ".orientation.x").as_double();
        wp.pose.orientation.y = get_parameter(cleaning_waypoint + ".orientation.y").as_double();
        wp.pose.orientation.z = get_parameter(cleaning_waypoint + ".orientation.z").as_double();
        wp.pose.orientation.w = get_parameter(cleaning_waypoint + ".orientation.w").as_double();
        // wp.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));
        cleaning_waypoints_[start_wp_name].push_back(wp);

        geometry_msgs::msg::TransformStamped wp_t;
        wp_t.header = wp.header;
        wp_t.child_frame_id = cleaning_waypoint;
        wp_t.transform.translation.x = wp.pose.position.x;
        wp_t.transform.translation.y = wp.pose.position.y;
        wp_t.transform.translation.z = wp.pose.position.z;
        wp_t.transform.rotation = wp.pose.orientation;
        tf_broadcaster_->sendTransform(wp_t);
      }
    }

    declare_parameter<std::string>("action_cost_plugin", "plansys2_actions_cost::MoveThroughPosesActionCostDuration");
    std::string action_cost_plugin = get_parameter("action_cost_plugin").as_string();
    move_action_cost_loader_.reset(new pluginlib::ClassLoader<plansys2_actions_cost::MoveThroughPosesActionCostBase>("plansys2_actions_cost", "plansys2_actions_cost::MoveThroughPosesActionCostBase"));
    try {
      move_action_cost_ =
        move_action_cost_loader_->createSharedInstance(
        action_cost_plugin);
      std::cerr << "MoveThroughPosesActionCost created" << std::endl;
    } catch (pluginlib::PluginlibException & ex) {
      std::cerr << "The plugin failed to load for some reason. Error: " << ex.what() << std::endl;
    }
    RCLCPP_INFO(get_logger(), "Clean created");
  }


  CallbackReturnT
  on_configure(const rclcpp_lifecycle::State & state)
  {
    CallbackReturnT result = plansys2_actions_clients::ActionObservedCostClient::on_configure(state);
    if(result != CallbackReturnT::SUCCESS)
    {
      return result;
    }
    RCLCPP_INFO(get_logger(), "Clean pre configured");
    move_action_cost_->initialize(std::dynamic_pointer_cast<plansys2_actions_clients::ActionObservedCostClient>(shared_from_this()));
    RCLCPP_INFO(get_logger(), "Clean post configured");

    return CallbackReturnT::SUCCESS; 
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    send_feedback(0.0, "Clean starting");

    navigation_action_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
      shared_from_this(),
      namespace_ + "/navigate_through_poses");

    bool is_action_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");

      is_action_server_ready =
        navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
    } while (!is_action_server_ready && rclcpp::ok());

    RCLCPP_INFO(get_logger(), "Navigation action server ready");

    for(const auto& arg: get_arguments())
    {
      RCLCPP_INFO(get_logger(), "Arg start: %s", arg.c_str());
    }
    auto start_cleaning_waypoint = get_arguments()[2];  // The goal is in the 3rd argument of the action
    RCLCPP_INFO(get_logger(), "Start cleaning from [%s]", start_cleaning_waypoint.c_str());
    
    if(cleaning_waypoints_.find(start_cleaning_waypoint) == cleaning_waypoints_.end())
    {
      RCLCPP_ERROR(get_logger(), "Waypoint not found, has to be managed by specialized arguments");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "Analyzing waypoints");
    for(const auto& wp: cleaning_waypoints_[start_cleaning_waypoint])
    {
      RCLCPP_INFO(get_logger(), "Waypoint x: %f, y: %f", wp.pose.position.x, wp.pose.position.y);
    }
    // waypoints_ = cleaning_waypoints_[start_cleaning_waypoint];
    
    // goal_pos_ = waypoints_;
    navigation_goal_.poses = cleaning_waypoints_[start_cleaning_waypoint];
    // TODO(@samu) to check if the goal is the last one
    dist_to_move = 10; //getDistance(navigation_goal_.poses., current_pos_.pose);

    auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();

    send_goal_options.feedback_callback = [this](
      NavigationGoalHandle::SharedPtr,
      NavigationFeedback feedback) {
        // plansys2::ActionExecutorClient::set_action_cost(feedback->distance_remaining, 0.0);
        send_feedback(
          std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),
          "Move running");
      };

    send_goal_options.result_callback = [this](auto) {
        // finish(true, 1.0, "Move completed");
        // compute the end_time and the duration end-start and update fluents
        rclcpp::Time end_time = now();
        auto action_duration = end_time - start_time_;
        double duration = action_duration.nanoseconds() * 1e-9;
        
        finish(true, 1.0, "Move completed", duration);

        RCLCPP_INFO(get_logger(), "Measured Action duration: %f", duration);

      };

    future_navigation_goal_handle_ =
      navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);

    return ActionExecutorClient::on_activate(previous_state);
  }

private:

  double getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2)
  {
    return sqrt(
      (pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
      (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
  }

  void do_work()
  {
  }
  
  void compute_action_cost(const plansys2_msgs::msg::ActionExecution::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Compute action cost");
    auto start_cleaning_waypoint = get_arguments()[2];  // The goal is in the 3rd argument of the action
    RCLCPP_INFO(get_logger(), "Start cleaning from [%s]", start_cleaning_waypoint.c_str());

    if(cleaning_waypoints_.find(start_cleaning_waypoint) == cleaning_waypoints_.end())
    {
      RCLCPP_ERROR(get_logger(), "Waypoint not found, has to be managed by specialized arguments");
      plansys2::ActionExecutorClient::set_action_cost(std::numeric_limits<double>::infinity(), 0.0);
    }
    auto goal = cleaning_waypoints_[start_cleaning_waypoint];

    move_action_cost_->compute_action_cost(goal, msg);
    // while(!move_action_cost_->is_action_cost_setted())
    // {
    //   this->set_action_cost(move_action_cost_->get_action_cost());
    // }
  } 

  std::map<std::string, std::vector<geometry_msgs::msg::PoseStamped>> cleaning_waypoints_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  std::shared_ptr<plansys2_actions_cost::MoveThroughPosesActionCostBase> move_action_cost_;
  std::shared_ptr<pluginlib::ClassLoader<plansys2_actions_cost::MoveThroughPosesActionCostBase>> move_action_cost_loader_; // {"plansys2_actions_cost", "plansys2_actions_cost::MoveActionCostBase"};
  
  std::string namespace_;

  using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>;
  using NavigationFeedback =
    const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback>;


  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr navigation_action_client_;
  std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  geometry_msgs::msg::PoseStamped current_pos_;
  // std::vector<geometry_msgs::msg::PoseStamped> goal_pos_;
  nav2_msgs::action::NavigateThroughPoses::Goal navigation_goal_;

  double dist_to_move;

  bool test_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CleanAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "clean"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
