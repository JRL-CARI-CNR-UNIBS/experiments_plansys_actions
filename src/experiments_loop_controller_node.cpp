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

#include <fstream>

class ExperimentsLoopController : public rclcpp::Node
{
public:
  ExperimentsLoopController()
  : rclcpp::Node("experiments_controller"), state_(STARTING)
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

  typedef enum {STARTING, INIT, EXECUTE, FINISH, RESET, RESETTED} StateType;

  StateType get_state()
  {
    return state_;
  }

  bool executing()
  {
    return executor_client_->execute_and_check_plan(); 
  }
  plansys2_msgs::msg::Plan get_executed_plan()
  {
    return execute_plan_;
  }

  bool is_finish()
  {
    bool is_finish = !executor_client_->execute_and_check_plan() && executor_client_->getResult();
    if(is_finish) {
      if(state_ == RESET) {
        state_ = RESETTED;
      }
      else {
        state_ = FINISH;
      }
    }
    return is_finish;
  }

  std::optional<plansys2_msgs::action::ExecutePlan::Result> get_execution_result()
  {
    return executor_client_->getResult();
  }

  bool reset_state_variables()
  {
    for(const auto & predicate: reset_predicates_)
    {
      if(!problem_expert_->removePredicate(plansys2::Predicate(predicate))) {
        RCLCPP_ERROR(get_logger(), "Predicate %s not removed.", predicate.c_str());
        return false;
      }
    }
    for(const auto & predicate: initial_predicates_)
    {
      if(!problem_expert_->addPredicate(plansys2::Predicate(predicate))) {
        RCLCPP_ERROR(get_logger(), "Predicate %s not added.", predicate.c_str());
        return false;
      }
    }
    return true;
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
  

  void plan_execute()
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
    execute_plan_ = plan.value();
    if (executor_client_->start_plan_execution(execute_plan_)) {
      state_ = EXECUTE;
    }
  }

  void reset_world_state()
  {
    if(reset_world_goal_.empty()) {
      return;
    }

    if(!problem_expert_->clearGoal()) {
      throw std::runtime_error("Goal of the problem file is not cleared correctly.");
    }
    if(!problem_expert_->setGoal(plansys2::Goal(reset_world_goal_.c_str()))) {
      throw std::runtime_error("Goal defined into param is not set correctly.");
    }
    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      std::cout << "Could not find plan to reach goal " <<
        parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
      return;
    }
    if (executor_client_->start_plan_execution(plan.value())) {
      state_ = RESET;
    }
  }

private:
  StateType state_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  std::vector<std::string> initial_predicates_;
  std::vector<std::string> reset_predicates_;
  std::string reset_world_goal_;
  plansys2_msgs::msg::Plan execute_plan_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExperimentsLoopController>();

  auto plan_execution_data_pub = node->create_publisher<plansys2_msgs::msg::PlanExecutionDataCollection>("plan_execution_data_collection", 10);

  node->declare_parameter<int>("n_execution", 1);
  int n_execution = node->get_parameter("n_execution").as_int();
  
  node->init();
  rclcpp::Time t_start;
  try{
    node->plan_execute();
    t_start = node->now();
  }
  catch(const std::exception & e)
  {
    RCLCPP_ERROR(node->get_logger(), e.what());
    return 0;
  }

  rclcpp::Rate rate(5);
  int plan_execution = 0;
  while (rclcpp::ok()) {
    if (node->is_finish()) {
    // if (!node->executing() && node->get_execution_result()) {
      if (node->get_execution_result().value().success) {
        RCLCPP_INFO(node->get_logger(), "Plan finished with success.");
        node->reset_state_variables();
        
        plan_execution++;

        if(plan_execution >= n_execution) {
          RCLCPP_INFO(node->get_logger(), "All executions finished.");
          break;
        }
        
        switch(node->get_state())
        {
          case ExperimentsLoopController::FINISH:
          {
            plansys2_msgs::msg::PlanExecutionDataCollection msg;
            msg.plan = node->get_executed_plan();
            msg.action_execution_status = node->get_execution_result().value().action_execution_status;
            msg.t_start = t_start;
            msg.t_end = node->now();
            plan_execution_data_pub->publish(msg);
          }

            try{
              RCLCPP_INFO(node->get_logger(), "Resetting the world state.");
              node->reset_world_state();
            }
            catch(const std::exception & e) {
              RCLCPP_ERROR(node->get_logger(), e.what());
              return 0;
            }        
          break;
          case ExperimentsLoopController::RESETTED:
            node->reset_state_variables(); // Again since the plan to achieve start init can contain also...
            try{
              node->plan_execute();
              t_start = node->now();
              RCLCPP_INFO(node->get_logger(), "Execute another plan.");
            }
            catch(const std::exception & e) {
              RCLCPP_ERROR(node->get_logger(), e.what());
              return 0;
            }
            break;
          default:
            RCLCPP_ERROR(node->get_logger(), "State not resetted. %d", node->get_state());
            return 0;
        }
      }
      else { // finished failed
        RCLCPP_INFO(node->get_logger(), "Plan finished with error");
        break;
      }
    }
    // executing
    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
