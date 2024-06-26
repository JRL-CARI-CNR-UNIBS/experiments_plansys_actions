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

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_action_clients/action_observed_cost_client.hpp"

#include "plansys2_actions_cost/move_action_cost_base.hpp"
#include "audio_common_msgs/action/tts.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2_ros/static_transform_broadcaster.h"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include <pluginlib/class_loader.hpp>

using namespace std::chrono_literals;

class AssistPerson : public plansys2_actions_clients::ActionObservedCostClient
{
public:
  AssistPerson()
  : plansys2_actions_clients::ActionObservedCostClient("move", 500ms) //: move_action_cost_loader_("plansys2_actions_cost", "plansys2_actions_cost::MoveActionCostBase")
  {
    RCLCPP_INFO(get_logger(), "AttendPerson created");

    tf_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    RCLCPP_INFO(get_logger(), "AttendPerson created");
    declare_parameter<std::string>("namespace", "");
    namespace_ = get_parameter("namespace").as_string();
    
    declare_parameter<std::vector<std::string>>("waypoints", std::vector<std::string>());
    auto waypoint_names = get_parameter("waypoints").as_string_array();
    for (const auto& name : waypoint_names) {
      geometry_msgs::msg::PoseStamped wp;
      wp.header.frame_id = "map";
      wp.header.stamp = now();
      // RCLCPP_INFO(get_logger(), "Waypoint: %s", name.c_str());
      declare_parameter<double>(name + ".position.x", 0.0);
      declare_parameter<double>(name + ".position.y", 0.0);
      declare_parameter<double>(name + ".position.z", 0.0);
      declare_parameter<double>(name + ".orientation.x", 0.0);
      declare_parameter<double>(name + ".orientation.y", 0.0);
      declare_parameter<double>(name + ".orientation.z", 0.0);
      declare_parameter<double>(name + ".orientation.w", 1.0);
      wp.pose.position.x = get_parameter(name + ".position.x").as_double();
      wp.pose.position.y = get_parameter(name + ".position.y").as_double();
      wp.pose.position.z = get_parameter(name + ".position.z").as_double();
      wp.pose.orientation.x = get_parameter(name + ".orientation.x").as_double();
      wp.pose.orientation.y = get_parameter(name + ".orientation.y").as_double();
      wp.pose.orientation.z = get_parameter(name + ".orientation.z").as_double();
      wp.pose.orientation.w = get_parameter(name + ".orientation.w").as_double();

      geometry_msgs::msg::TransformStamped wp_t;
      wp_t.header = wp.header;
      wp_t.child_frame_id = std::string(name);
      wp_t.transform.translation.x = wp.pose.position.x;
      wp_t.transform.translation.y = wp.pose.position.y;
      wp_t.transform.translation.z = wp.pose.position.z;
      wp_t.transform.rotation = wp.pose.orientation;
      tf_broadcaster_->sendTransform(wp_t);

      waypoints_[name] = wp;
      // RCLCPP_INFO(get_logger(), "Waypoint: %s, x: %f, y: %f", name.c_str(), wp.pose.position.x, wp.pose.position.y);
    }
    
    declare_parameter<std::string>("action_cost_plugin", "plansys2_actions_cost::MoveActionCostLength");
    std::string action_cost_plugin = get_parameter("action_cost_plugin").as_string();
    move_action_cost_loader_.reset(new pluginlib::ClassLoader<plansys2_actions_cost::MoveActionCostBase>("plansys2_actions_cost", "plansys2_actions_cost::MoveActionCostBase"));
    try {
      move_action_cost_ =
        move_action_cost_loader_->createSharedInstance(
        action_cost_plugin);
      std::cerr << "MoveActionCostLength created" << std::endl;
    } catch (pluginlib::PluginlibException & ex) {
      std::cerr << "The plugin failed to load for some reason. Error: " << ex.what() << std::endl;
    }
    RCLCPP_INFO(get_logger(), "AttendPerson created");
  }

  void current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    current_pos_.header = msg->header;
    current_pos_.pose = msg->pose.pose;
  }

  CallbackReturnT
  on_configure(const rclcpp_lifecycle::State & state)
  {
    CallbackReturnT result = plansys2_actions_clients::ActionObservedCostClient::on_configure(state);
    if(result != CallbackReturnT::SUCCESS)
    {
      return result;
    }
    RCLCPP_INFO(get_logger(), "AttendPerson pre configured");
    move_action_cost_->initialize(std::dynamic_pointer_cast<plansys2_actions_clients::ActionObservedCostClient>(shared_from_this()));
    RCLCPP_INFO(get_logger(), "AttendPerson post configured");

    return CallbackReturnT::SUCCESS; 
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    send_feedback(0.0, "AttendPerson starting");

    navigation_action_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      shared_from_this(),
      namespace_ + "/navigate_to_pose");

    bool is_action_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");

      is_action_server_ready =
        navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
    } while (!is_action_server_ready && rclcpp::ok());

    RCLCPP_INFO(get_logger(), "Navigation action server ready");

    auto wp_to_navigate = get_arguments()[3];  // The goal is in the 3rd argument of the action
    RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());

    goal_pos_ = waypoints_[wp_to_navigate];
    navigation_goal_.pose = goal_pos_;

    dist_to_move = getDistance(goal_pos_.pose, current_pos_.pose);

    auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    send_goal_options.feedback_callback = [this](
      NavigationGoalHandle::SharedPtr,
      NavigationFeedback feedback) {
        // plansys2::ActionExecutorClient::set_action_cost(feedback->distance_remaining, 0.0);
        // send_feedback(
        //   std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),
        //   "Move running");
      };

    send_goal_options.result_callback = [this](auto) {
        // finish(true, 1.0, "Move completed");
        // compute the end_time and the duration end-start and update fluents
        rclcpp::Time end_time = now();
        auto action_duration = end_time - start_time_;
        double duration = action_duration.nanoseconds() * 1e-9;
        
        finish(true, 1.0, "Move completed", duration);

        RCLCPP_INFO(get_logger(), "Measured Action duration: %f", duration);
        // std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
        // auto args = get_arguments();
        // for(auto & arg : args)
        // {
        //   RCLCPP_INFO(get_logger(), "Arg: %s", arg.c_str());
        // }
        // problem_expert_->updateFunction(plansys2::Function("(= (move_duration " + args[0] + " " + args[1] + " " + args[2] + ") " + std::to_string(duration) + ")"));
        // RCLCPP_INFO(get_logger(), "Updated move_duration");
        // problem_expert_->updateFunction(plansys2::Function("(= (move_duration r2d2 wp4 wp_control) 1000)"));
        // update_knowledge();
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
    auto wp_to_navigate = get_arguments()[3];  // The goal is in the 3rd argument of the action
    
    RCLCPP_INFO(get_logger(), "Computing cost to [%s]", wp_to_navigate.c_str()); 
    if(waypoints_.find(wp_to_navigate) == waypoints_.end())
    {
      RCLCPP_ERROR(get_logger(), "Waypoint not found, has to be managed by specialized arguments");
      plansys2::ActionExecutorClient::set_action_cost(std::numeric_limits<double>::infinity(), 0.0);
      // TODO(@samuele): It is not enough to set the cost, we have to send a response! Check this!
    }
    auto goal_pose = waypoints_[wp_to_navigate];
    /* Send waypoints to tf broadcaster */
    std::cerr << "Sending waypoint to tf broadcaster" << std::endl;
    geometry_msgs::msg::TransformStamped wp_t;
    wp_t.header = goal_pose.header;
    wp_t.child_frame_id = std::string(get_name()) + "_" + wp_to_navigate;
    wp_t.transform.translation.x = goal_pose.pose.position.x;
    wp_t.transform.translation.y = goal_pose.pose.position.y;
    wp_t.transform.translation.z = goal_pose.pose.position.z;
    wp_t.transform.rotation = goal_pose.pose.orientation;
    tf_broadcaster_->sendTransform(wp_t);

    move_action_cost_->compute_action_cost(goal_pose, msg);

  } 

  std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<plansys2_actions_cost::MoveActionCostBase> move_action_cost_;
  std::shared_ptr<pluginlib::ClassLoader<plansys2_actions_cost::MoveActionCostBase>> move_action_cost_loader_; // {"plansys2_actions_cost", "plansys2_actions_cost::MoveActionCostBase"};
  std::string namespace_;
  using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using NavigationFeedback =
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;


  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  geometry_msgs::msg::PoseStamped current_pos_;
  geometry_msgs::msg::PoseStamped goal_pos_;
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;

  double dist_to_move;

  bool test_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AssistPerson>();

  node->set_parameter(rclcpp::Parameter("action_name", "attend_person"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
