import pytest
from unittest.mock import MagicMock, patch
from plansys2_msgs.srv import GetDomain, GetProblem, RetrievePlan
from plansys2_msgs.msg import Plan, PlanItem
from std_srvs.srv import Trigger
from experiments_plansys_actions.task_plan_uncertainty import TaskPlanUncertainty  # replace with the correct module name
import rclpy 
import os 

@pytest.mark.dependency(name="setUp")
def test_setup():
    rclpy.init()


@pytest.mark.dependency(depends=["test_setup"])
@pytest.fixture
def mock_node():
  
    current_dir = os.path.dirname(os.path.abspath(__file__))
    domain_path = os.path.join(current_dir, '/pddl/domain.pddl')
    problem_path = os.path.join(current_dir, '/pddl/problem.pddl')  

    print(domain_path)
    print(problem_path)

    # Initialize the node
    node = TaskPlanUncertainty()

    # Mock for get_domain_client
    node.get_domain_client = MagicMock()
    domain_response = GetDomain.Response()
    with open(domain_path, 'r') as file:
        domain_response.domain = file.read()
    node.get_domain_client.call.return_value = domain_response

    # Mock for get_problem_client
    node.get_problem_client = MagicMock()
    problem_response = GetProblem.Response()
    with open(problem_path, 'r') as file:
        problem_response.problem = file.read()
    node.get_problem_client.call.return_value = problem_response

    # Mock for retrieve_plan_client
    node.retrieve_plan_client = MagicMock()
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
    
    plan_response = RetrievePlan.Response()
    plan_response.success = True
    plan_response.plan = plan_example
    node.retrieve_plan_client.call.return_value = plan_response

    return node

def test_plan_uncertainty_success(mock_node):
    # Prepare the request and response
    request = Trigger.Request()
    response = Trigger.Response()

    # Call the plan_uncertainty service
    response = mock_node.compute_uncertainty(request, response)

    # Verify that the response is positive
    assert response.success is True
    assert response.message == "Uncertainty computed"

def test_plan_uncertainty_empty_domain(mock_node):
    # Modify the response to test empty domain
    mock_node.get_domain_client.call.return_value.domain = ""

    # Prepare the request and response
    request = Trigger.Request()
    response = Trigger.Response()

    # Call the plan_uncertainty service
    response = mock_node.compute_uncertainty(request, response)

    # Verify that empty domain returns an error
    assert response.success is False
    assert response.message == "Domain empty"

def test_plan_uncertainty_empty_problem(mock_node):
    # Modify the response to test empty problem
    mock_node.get_problem_client.call.return_value.problem = ""

    # Prepare the request and response
    request = Trigger.Request()
    response = Trigger.Response()

    # Call the plan_uncertainty service
    response = mock_node.compute_uncertainty(request, response)

    # Verify that empty problem returns an error
    assert response.success is False
    assert response.message == "Problem empty"

def test_plan_uncertainty_plan_error(mock_node):
    # Modify the response of the plan service to simulate an error
    mock_node.retrieve_plan_client.call.return_value.success = False
    mock_node.retrieve_plan_client.call.return_value.error_info = "Plan retrieval failed"

    # Prepare the request and response
    request = Trigger.Request()
    response = Trigger.Response()

    # Call the plan_uncertainty service
    response = mock_node.compute_uncertainty(request, response)

    # Verify that an error in plan retrieval returns an error message
    assert response.success is False
    assert response.message == "Plan retrieval failed"
