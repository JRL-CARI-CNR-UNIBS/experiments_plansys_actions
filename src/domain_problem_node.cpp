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

#include <plansys2_pddl_parser/Utils.h>

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_msgs/action/execute_plan.hpp"
#include "plansys2_msgs/msg/plan_execution_data_collection.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp/service.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <fstream>
#include <random>

using namespace std::chrono_literals;

class DomainProblemNode : public rclcpp::Node
{
public:
  DomainProblemNode()
  : rclcpp::Node("domain_problem_node")
  {
    declare_parameter<std::string>("problem", std::string(""));
    declare_parameter<std::string>("goal", std::string(""));

    declare_parameter<std::vector<std::string>>("initial_predicates", std::vector<std::string>());
    declare_parameter<std::vector<std::string>>("reset_predicates", std::vector<std::string>());  
    declare_parameter<std::string>("reset_world_goal", std::string(""));
    get_parameter("initial_predicates", initial_predicates_);
    get_parameter("reset_predicates", reset_predicates_);
    get_parameter("reset_world_goal", reset_world_goal_);
  }
  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    init_knowledge();
  }

  void init_knowledge()
  {
    std::string problem;
    get_parameter_or("problem", problem, std::string(""));
    problem_expert_->addProblem(problem);
  }
  
private:

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;

  std::vector<std::string> initial_predicates_;
  std::vector<std::string> reset_predicates_;
  std::string reset_world_goal_;
  plansys2_msgs::msg::Plan execute_plan_;

  std::string domain_, problem_;
};

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);

  auto node = std::make_shared<DomainProblemNode>();
  auto initialize_knowledge_client = node->create_client<std_srvs::srv::Trigger>("initialize_knowledge");
  
  
  node->init();
  if(initialize_knowledge_client->wait_for_service(1s)) {
    auto future_result = initialize_knowledge_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    rclcpp::spin_until_future_complete(node->get_node_base_interface(), future_result);
    auto result = future_result.get();
    if(!result->success) {
      RCLCPP_ERROR(node->get_logger(), "Service failed.");
      return 0;
    }
  }
  RCLCPP_INFO(node->get_logger(), "Knowledge initialized.");
  rclcpp::Rate rate(5);
  while(rclcpp::ok()) {
    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
