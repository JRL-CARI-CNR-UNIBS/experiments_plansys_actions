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

#include <memory>

#include "geometry_msgs/msg/twist.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_action_clients/action_observed_cost_client.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;

class Patrol : public plansys2_actions_clients::ActionObservedCostClient
{
public:
  Patrol()
  : plansys2_actions_clients::ActionObservedCostClient("patrol", 100ms)
  {
    declare_parameter<std::string>("namespace", "");
    namespace_ = get_parameter("namespace").as_string();
    first_iteration_ = true;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    progress_ = 0.0;

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(namespace_ + "/cmd_vel", 10);
    cmd_vel_pub_->on_activate();

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    cmd_vel_pub_->on_deactivate();
    first_iteration_ = true;
    return ActionExecutorClient::on_deactivate(previous_state);
  }

private:
  void do_work()
  {
    if(first_iteration_){
      start_patrol_time_ = now();
      first_iteration_ = false;
    }
    
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = 0.5;
    cmd_vel_pub_->publish(cmd);
    
    rclcpp::Duration time_between_start = now() - start_patrol_time_;
    if(time_between_start > 10s){
      cmd.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd);
      // finish(true, 1.0, "Patrol completed");
      auto action_duration = now() - get_start_time();
      double duration = action_duration.nanoseconds() * 1e-9;
      finish(true, 1.0, "Patrol completed", duration);
      first_iteration_ = true;
      return;
    }
    else{
      progress_ = time_between_start.seconds() / 10.0;
      send_feedback(progress_, "Patrol running");
    }
  }

  float progress_;
  bool first_iteration_;
  std::string namespace_;
  rclcpp::Time start_patrol_time_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();

  node->set_parameter(rclcpp::Parameter("action_name", "patrol"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
