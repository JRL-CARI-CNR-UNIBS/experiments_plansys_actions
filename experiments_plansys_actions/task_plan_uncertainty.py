#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from plansys2_msgs.srv import GetDomain, GetProblem, RetrievePlan
from plansys2_msgs.msg import Plan
from plansys2_knowledge_base_msgs.srv import GetFluentMetadata
from plansys2_knowledge_base_msgs.msg import FluentMetadata

from unified_planning.io import PDDLReader
from unified_planning.plans.plan import PlanKind
from unified_planning.plans.stn_plan import STNPlanNode, TimepointKind
from unified_planning.model.problem import Problem
from unified_planning.plans.plan import Plan

from ortools.linear_solver import pywraplp
from abc import ABC, abstractmethod
import importlib
from typing import List, Tuple
import numpy as np

MAX_TIME = 1000
EPS = 1e-1

class PlanUncertaintyComputation(ABC):
    @abstractmethod
    def compute_uncertainty(self, parsed_plan: Plan, parsed_problem: Problem):
        pass
    
class DefaultPlanUncertaintyComputation(PlanUncertaintyComputation):
    def __init__(self):

        self.plan_uncertainty_node = Node('plan_uncertainty_node')

        callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        self.get_fluent_metadata_client = self.plan_uncertainty_node.create_client(GetFluentMetadata, 
                                                                                   'get_fluent_metadata', 
                                                                                   callback_group=callback_group)
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.plan_uncertainty_node)
    
    def _reconstruct_fluent_from_action(self, action_instance):
        pass
    
    def _retrieve_fluent_metadata(self, fluent_name: str, fluent_params: List[str]) -> Tuple[float, float]:
        get_fluent_metadata_request = GetFluentMetadata.Request()
        get_fluent_metadata_request.fluent_name = fluent_name
        get_fluent_metadata_request.parameters = fluent_params

        future = self.get_fluent_metadata_client.call_async(get_fluent_metadata_request)
        self.executor.spin_until_future_complete(future, timeout_sec=1)
        metadata = future.result()
        fluent_metadata = metadata.fluent_metadata
        
        return fluent_metadata.value, fluent_metadata.variance


    def compute_uncertainty(self, parsed_plan: Plan, parsed_problem: Problem):
        if not self.get_fluent_metadata_client.wait_for_service(timeout_sec = 2):
            return None, None
        stn_plan = parsed_plan.convert_to(PlanKind.STN_PLAN, parsed_problem)
        constraints = stn_plan.get_constraints()

        model = pywraplp.Solver.CreateSolver('GLOP')
        
        start_plan = STNPlanNode(TimepointKind.GLOBAL_START)
        end_plan = STNPlanNode(TimepointKind.GLOBAL_END)
        if start_plan not in constraints:
            constraints[start_plan] = []
        if end_plan not in constraints:
            constraints[end_plan] = []

        time_vars = {}
        action_duration = {}
        action_variance = {}

        # Retrieve the action duration and variance
        for node in constraints:
            for (start, action_instance, duration) in parsed_plan.timed_actions:
                if str(action_instance) in str(node):
                    # duration_interval = action_instance.action.duration
                    upper_time_bound = action_instance.action.duration.upper # The upper bound of the duration is a fluent
                    corrisponging_fluent = str(upper_time_bound).split('(')[0] # Extract the name of the fluent duration_move(...), just duration_move
                    
                    params = []
                    for param in action_instance.actual_parameters:
                        params.append(str(param))
                    fluent_params = params

                    # request of GetFluentMetadata                  
                    expected_value, variance = self._retrieve_fluent_metadata(corrisponging_fluent, fluent_params)
                    action_duration[node] = expected_value
                    action_variance[node] = variance
                    break       
        
        # monte-carlo approach
        plan_durations = []

        for _ in range(500):
            # Create a new model for each iteration
            model = pywraplp.Solver.CreateSolver('GLOP')
            
            # Create time variables for each node in the constraints
            time_vars = {}
            for node in constraints:
                time_vars[node] = model.NumVar(0, MAX_TIME, f"time_{node}")

            # Add constraints between nodes based on variance and bounds
            for node_A, edges in constraints.items():
                node_a_variance = None
                if node_A in action_duration:
                    node_a_std_dev = np.sqrt(action_variance[node_A])
                    node_a_variance = np.random.normal(0, node_a_std_dev)

                for lower_bound, upper_bound, node_B in edges:
                    if lower_bound is not None:
                        if float(lower_bound) < EPS or node_A not in action_duration:
                            model.Add(time_vars[node_B] - time_vars[node_A] >= float(lower_bound))
                        else:
                            model.Add(time_vars[node_B] - time_vars[node_A] >= float(lower_bound) + node_a_variance)

                    if upper_bound is not None:
                        if float(upper_bound) < EPS or node_A not in action_duration:
                            model.Add(time_vars[node_B] - time_vars[node_A] <= float(upper_bound))
                        else:
                            model.Add(time_vars[node_B] - time_vars[node_A] <= float(upper_bound) + node_a_variance)
            
            # Minimize the time of the end plan
            model.Minimize(time_vars[end_plan])
            status = model.Solve()

            if status == pywraplp.Solver.OPTIMAL:
                plan_duration = time_vars[end_plan].solution_value()
                plan_durations.append(plan_duration)
            else:
                print('No optimal solution found.')
        mean = np.mean(plan_durations)
        std_dev = np.std(plan_durations)

        if len(plan_durations) > 0:
            self.plan_uncertainty_node.get_logger().info(f"Mean: {mean}, Std Dev: {std_dev}")
            return mean, std_dev
        self.plan_uncertainty_node.get_logger().warning("Not enough data to compute uncertainty")
        return None, None
    
class TaskPlanUncertainty(Node):

    def __init__(self):
        super().__init__('task_plan_uncertainty_node')

        self.declare_parameter('plan_uncertainty_computation_module', '')

        plan_uncertainty_computation_module = self.get_parameter('plan_uncertainty_computation_module').get_parameter_value().string_value
        if plan_uncertainty_computation_module:
            module_name, class_name = plan_uncertainty_computation_module.rsplit('.', 1)
            module = importlib.import_module(module_name)
            uncertainty_computation_class = getattr(module, class_name)
        else:
            uncertainty_computation_class = DefaultPlanUncertaintyComputation

        self.uncertainty_computation = uncertainty_computation_class()

        internal_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        self.get_domain_client = self.create_client(srv_type=GetDomain, srv_name='/domain_expert/get_domain', callback_group=internal_callback_group)
        self.get_problem_client = self.create_client(srv_type=GetProblem, srv_name='/problem_expert/get_problem', callback_group=internal_callback_group)
        self.retrieve_plan_client = self.create_client(srv_type=RetrievePlan, srv_name='/planner/retrieve_plan', callback_group=internal_callback_group)

        if not self.get_domain_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/get_domain service not available, exiting...')
            return
        if not self.get_problem_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/get_problem service not available, exiting...')
            return
        if not self.retrieve_plan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/retrieve_plan service not available, exiting...')
            return 
        
        self.compute_uncertainty_service = self.create_service(Trigger, 'plan_uncertainty', self.compute_uncertainty)

        self.pddl_reader = PDDLReader()
        self.get_logger().info('Task Plan Uncertainty node is ready')

    def _prepare_plan_string(self, plan: Plan) -> str:
        plan_str_upf_compatible = ""
        for item in plan.items:
            plan_str_upf_compatible += f"{item.time}: {item.action} [{item.duration}]\n"
        return plan_str_upf_compatible

    def compute_uncertainty(self, request, response):
        self.get_logger().info('Computing uncertainty')
        domain_request = GetDomain.Request()
        domain_response = self.get_domain_client.call(domain_request)
        domain_str = domain_response.domain
        if not domain_str:
            response.success = False
            response.message = "Domain empty"
            return response
        self.get_logger().info(f'Domain: {domain_str}')
        
        self.get_logger().info('Getting problem')
        problem_request = GetProblem.Request()
        problem_response = self.get_problem_client.call(problem_request)
        problem_str = problem_response.problem
        if not problem_str:
            response.success = False
            response.message = "Problem empty"
            return response
        self.get_logger().info(f'Problem: {problem_str}')
        try:
            parsed_problem = self.pddl_reader.parse_problem_string(domain_str, problem_str)
        except Exception as e:
            response.success = False
            response.message = "Error parsing problem"
            return response
        self.get_logger().info(f'Parsed problem: {parsed_problem}')

        self.get_logger().info('Getting plan')
        plan_request = RetrievePlan.Request()
        plan_response = self.retrieve_plan_client.call(plan_request)
        if not plan_response.success:
            response.success = False
            response.message = plan_response.error_info
            return response
        plan: Plan = plan_response.plan

        plan_str_upf_compatible = self._prepare_plan_string(plan)
        try:
            parsed_plan = self.pddl_reader.parse_plan_string(problem=parsed_problem, plan_str=plan_str_upf_compatible)
        except Exception as e:
            response.success = False
            response.message = "Error parsing plan"
            return response
        self.get_logger().info(f'Parsed plan: {parsed_plan}')

        expected_duration, standard_deviation = self.uncertainty_computation.compute_uncertainty(parsed_plan, parsed_problem)
        if standard_deviation:
            response.success = True
            response.message = "Uncertainty computed"
            return response
        response.success = False
        response.message = "Error computing uncertainty"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TaskPlanUncertainty()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        if rclpy.ok():
            executor.remove_node(node)
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
