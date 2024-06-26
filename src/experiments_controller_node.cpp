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

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <fstream>

class ExperimentsController : public rclcpp::Node
{
public:
  ExperimentsController()
  : rclcpp::Node("experiments_controller"), state_(STARTING)
  {
    declare_parameter<std::string>("problem", std::string(""));
    declare_parameter<std::string>("goal", std::string(""));
  }

  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge();
    state_ = INIT;
  }

  void init_knowledge()
  {
    std::string problem;
    get_parameter_or("problem", problem, std::string(""));

    problem_expert_->addProblem(problem);
    
    // std::string problem_file = "/tmp/updated_problem.pddl";
    // if(problem_file.empty())
    // {
    //   std::cerr << "Empty problem." << std::endl;
    //   return;
    // }

    // std::ifstream problem_ifs(problem_file);
    // std::string problem_str((
    //     std::istreambuf_iterator<char>(problem_ifs)),
    //   std::istreambuf_iterator<char>());
    // if (problem_str.empty()) {
    //   std::cerr << "Empty problem." << std::endl;
    //   return;
    // }
    // problem_expert_->updateFunctionsFromUpdatedProblem(problem_str);

    // std::vector<plansys2::Function> functions = problem_expert_->updateFunction(file_path);
    // for(auto function: functions)
    // {
    //   problem_expert_->updateFunction(function);
    // }
    // problem_expert_->updateFunction(plansys2::Function("(= (move_duration r2d2 wp4 wp_control) 1000)"));
    // domain_expert_->getFunctions();
  }
  

  void step()
  {
    std::string goal;

    get_parameter_or("goal", goal, std::string(""));
    if(!goal.empty())
    {
      if(!problem_expert_->clearGoal())
      {
        throw std::runtime_error("Goal of the problem file is not cleared correctly.");
      }
      if(!problem_expert_->setGoal(plansys2::Goal(goal.c_str())))
      {
        throw std::runtime_error("Goal defined into param is not set correctly.");
      }
      RCLCPP_INFO(get_logger(), "Goal of the problem is overritten by the param.");
    }

    // Compute the plan
    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      std::cout << "Could not find plan to reach goal " <<
        parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
      return;
    }
    
    RCLCPP_INFO(get_logger(), "Plan:");
    for(const auto & item: plan.value().items)
    {
      RCLCPP_INFO(get_logger(), "[%f] Action: %s, Duration: %f", item.time, item.action.c_str(), item.duration);
    }
    // Execute the plan
    if (executor_client_->start_plan_execution(plan.value())) {
      state_ = EXECUTE;
    }

  }

private:
  typedef enum {STARTING, INIT, EXECUTE, FINISH} StateType;
  StateType state_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExperimentsController>();
  
  node->init();
  try{
    node->step();
  }
  catch(const std::exception & e)
  {
    RCLCPP_ERROR(node->get_logger(), e.what());
    return 0;
  }
  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    // node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
