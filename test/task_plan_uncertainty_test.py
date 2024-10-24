import pytest
from unittest.mock import MagicMock, patch
from plansys2_msgs.srv import GetDomain, GetProblem, RetrievePlan
from plansys2_msgs.msg import Plan, PlanItem
import rclpy.executors
from std_srvs.srv import Trigger
from experiments_plansys_actions.task_plan_uncertainty import TaskPlanUncertainty  # replace with the correct module name
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from plansys2_knowledge_base_msgs.srv import GetFluentMetadata, GetPlanMetadata
from plansys2_knowledge_base_msgs.msg import FluentMetadata

import rclpy 
import os 

from unified_planning.io import PDDLReader
from unified_planning.plans.plan import PlanKind
from unified_planning.plans.stn_plan import STNPlanNode, TimepointKind
from unified_planning.model.problem import Problem

import numpy as np

class FakeKnowledgeBase(Node):
    def __init__(self):
        super().__init__('fake_knowledge_base')
        self.get_fluent_metadata_client = self.create_service(GetFluentMetadata, '/get_fluent_metadata', self.get_fluent_metadata_callback)

    def get_fluent_metadata_callback(self, request, response):
        fluent_name = request.fluent_name
        fluent_params = request.parameters

        metadata = FluentMetadata(fluent_name=fluent_name, 
                                  parameters=fluent_params, 
                                  value=max(np.random.uniform(8,13),0), 
                                  variance=2.0, count=1)
        response.fluent_metadata = metadata
        
        response.success = True
        return response

class FakePlansysNode(Node):
    def __init__(self):
        super().__init__('fake_plansys_node')
        self.get_domain_client = self.create_service(GetDomain, '/domain_expert/get_domain', self.get_domain_callback)
        self.get_problem_client = self.create_service(GetProblem, '/problem_expert/get_problem', self.get_problem_callback)
        self.retrieve_plan_client = self.create_service(RetrievePlan, '/planner/retrieve_plan', self.retrieve_plan_callback)
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
    def get_domain_callback(self, request, response):
        domain_path = os.path.join(self.current_dir, 'pddl/domain.pddl')
        with open(domain_path, 'r') as file:
            response.domain = file.read()
        response.success = True
        return response
    def get_problem_callback(self, request, response):
        problem_path = os.path.join(self.current_dir, 'pddl/problem.pddl')
        with open(problem_path, 'r') as file:
            response.problem = file.read()
        response.success = True
        return response
    def retrieve_plan_callback(self, request, response):
        plan_example = Plan()
        plan_example.items.append(PlanItem(time=0.0, action="(move robot1 start_wp_robot1 clean1)", duration=8.0))
        plan_example.items.append(PlanItem(time=0.0, action="(move robot2 start_wp_robot2 hall)", duration=12.0))
        plan_example.items.append(PlanItem(time=8.00100040435791, action="(clean robot1 room1 clean1)", duration=10.100000381469727))
        plan_example.items.append(PlanItem(time=12.00100040435791, action="(patrol robot2 hall)", duration=5.400000095367432))
        plan_example.items.append(PlanItem(time=17.402000427246094, action="(move robot2 hall entrance)", duration=6.0))
        plan_example.items.append(PlanItem(time=18.101999282836914, action="(move robot1 clean1 clean2)", duration=9.0))
        plan_example.items.append(PlanItem(time=23.402999877929688, action="(attend_person robot2 persona entrance sofa)", duration=5.0))
        plan_example.items.append(PlanItem(time=27.10300064086914, action="(clean robot1 room2 clean2)", duration=10.100000381469727))
        plan_example.items.append(PlanItem(time=28.40399932861328, action="(move robot2 sofa clean3)", duration=9.0))
        plan_example.items.append(PlanItem(time=37.20399856567383, action="(move robot1 clean2 clean1)", duration=9.0))
        plan_example.items.append(PlanItem(time=37.404998779296875, action="(clean robot2 room3 clean3)", duration=10.199999809265137))
        response.plan = plan_example
        response.success = True
        return response
        
@pytest.mark.dependency(name="setUp")
def test_setup():
    rclpy.init()

@pytest.fixture
def fake_plansys():
    return FakePlansysNode()

@pytest.fixture
def fake_knowledge():
    return FakeKnowledgeBase()

@pytest.mark.dependency(name="uncertainty_with_retrieve", depends=["setUp"])
def test_plan_uncertainty_success(fake_plansys, fake_knowledge):
    executor = MultiThreadedExecutor()
    executor.add_node(fake_plansys)
    executor.add_node(fake_knowledge)

    task_plan_unc_node = TaskPlanUncertainty()
    executor.add_node(task_plan_unc_node)

    request = Trigger.Request()

    node = rclpy.create_node('fake_node')
    client = node.create_client(Trigger, 'retrieve_plan_compute_uncertainty')

    # Add the client node to the executor
    executor.add_node(node)

    # # Wait for the service to be available with a timeout
    if not client.wait_for_service(timeout_sec=5):
        pytest.fail("Service not available")

    # Call the service asynchronously with a timeout
    future = client.call_async(request)
    
    try:
        executor.spin_until_future_complete(future, timeout_sec=10)
    except Exception as e:
        pytest.fail(f"Service call failed: {e}")

    # Check the result
    response = future.result()
    print(response)
    assert response is not None
    assert response.success is True
    assert response.message == "Uncertainty computed"

    # Shutdown rclpy
    # rclpy.shutdown()

@pytest.mark.dependency(name="uncertainty_directly", depends=["setUp", "uncertainty_with_retrieve"])
def test_plan_uncertainty_directly(fake_plansys, fake_knowledge):
    executor = MultiThreadedExecutor()
    # executor.add_node(fake_plansys)
    executor.add_node(fake_knowledge)

    task_plan_unc_node = TaskPlanUncertainty()
    executor.add_node(task_plan_unc_node)

    request = GetPlanMetadata.Request()

    domain = fake_plansys.get_domain_callback(GetDomain.Request(), GetDomain.Response()).domain
    problem = fake_plansys.get_problem_callback(GetProblem.Request(), GetProblem.Response()).problem
    plan = fake_plansys.retrieve_plan_callback(RetrievePlan.Request(), RetrievePlan.Response()).plan

    request.domain = domain
    request.problem = problem
    request.plan = plan
    
    node = rclpy.create_node('fake_node')
    client = node.create_client(GetPlanMetadata, 'plan_uncertainty')

    # Add the client node to the executor
    executor.add_node(node)

    # # Wait for the service to be available with a timeout
    if not client.wait_for_service(timeout_sec=5):
        pytest.fail("Service not available")

    # Call the service asynchronously with a timeout
    future = client.call_async(request)
    
    try:
        executor.spin_until_future_complete(future, timeout_sec=10)
    except Exception as e:
        pytest.fail(f"Service call failed: {e}")

    # Check the result
    response = future.result()
    print(response)
    assert response is not None
    assert response.success is True
    assert response.error_info == "Uncertainty computed"

    # Shutdown rclpy
    rclpy.shutdown()