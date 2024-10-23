from unified_planning.plot import plot_plan
from unified_planning.plans.stn_plan import STNPlanNode, TimepointKind
from unified_planning.engines.results import PlanGenerationResultStatus
from unified_planning.environment import get_environment
from unified_planning.io import PDDLReader, PDDLWriter
from unified_planning.plot import plot_plan
from unified_planning.shortcuts import OneshotPlanner  # OptimalityGuarantee
from unified_planning.plans.plan import PlanKind
from ortools.sat.python import cp_model
from fractions import Fraction
from unified_planning.plans.time_triggered_plan import TimeTriggeredPlan,  get_partial_order_plan
from unified_planning.plans import SequentialPlan, PartialOrderPlan, ActionInstance
# from unified_planning.model.expression import 
domain_path = "/home/kalman/projects/turtlebot_ws/src/experiments_plansys_actions/pddl/tamer/domain.pddl"
problem_path = "/home/kalman/projects/turtlebot_ws/src/experiments_plansys_actions/pddl/tamer/problem.pddl"

reader = PDDLReader()
try:
  parsed_problem = reader.parse_problem(domain_path, problem_path)
except Exception as e:
  raise e
# print(type(parsed_problem.actions))
# for ac in parsed_problem.actions:
#     ac.
#     print(type(ac))
solver = "tamer"
with OneshotPlanner(name=solver) as planner:
    result = planner.solve(parsed_problem)

plan = result.plan
example_plan = "0.0: (move robot1 start_wp_robot1 clean1) [8.0]\n\
0.0: (move robot2 start_wp_robot2 clean2) [11.0]\n\
8.01: (clean robot1 room1 clean1) [10.1]\n\
11.01: (clean robot2 room2 clean2) [10.2]\n\
18.11: (move robot1 clean1 clean3) [12.0]\n\
21.21: (move robot2 clean2 hall) [15.0]\n\
30.12: (clean robot1 room3 clean3) [10.1]\n\
36.22: (patrol robot2 hall) [5.4]\n\
40.22: (move robot1 clean3 entrance) [11.0]\n\
51.23: (attend_person robot1 persona entrance sofa) [5.0]\n"
#TODO (TOGLIERE LE VIRGOLE E LE TONDE SISTEMARLE)
print(example_plan)
from unified_planning.io.utils import parse_string
plan = reader.parse_plan_string(problem=parsed_problem, plan_str=example_plan)

# plan_mock = TimeTriggeredPlan()
# parsed_problem.actions[0]
# print(parsed_problem.actions[0])
# action_instance = ActionInstance(parsed_problem.actions[0], ('robo1', 'start_wp_1', 'hall'))
# plan_example = TimeTriggeredPlan([(Fraction(1,1000), action_instance, Fraction(10,1))])
# action_instance = ActionInstance(parsed_problem.actions[0], {})
# start_action_durations = plan.timed_actions # actioninstance
# for timed_action in start_action_durations:
#     action_instance = timed_action[1]
#     action = action_instance.action
#     action_name = action.name
#     action_parameters = action.parameters # paramters
#     params = action_instance.actual_parameters
#     # print(action)
#     # print(action_name)
#     print(action_parameters)
#     # print(params)
stn_plan = plan.convert_to(PlanKind.STN_PLAN,parsed_problem)

from ortools.linear_solver import pywraplp

# # Crea il solver per la programmazione lineare continua

# # Estrarre i vincoli dallo STN plan
constraints = stn_plan.get_constraints()

# # Dizionario che mappa ogni nodo STN a una variabile temporale
time_vars = {}

start_plan = STNPlanNode(TimepointKind.GLOBAL_START)
end_plan = STNPlanNode(TimepointKind.GLOBAL_END)
if start_plan not in constraints:
    constraints[start_plan] = []
if end_plan not in constraints:
    constraints[end_plan] = []

variances = {}
nominal_value = {}
# Crea variabili per ciascun nodo

    
model = pywraplp.Solver.CreateSolver('GLOP')
action_duration = {}
action_variance = {}
for node in constraints:
    time_vars[node] = model.NumVar(0, 1000, f"time_{node}")
    for (start, action_instance, duration) in plan.timed_actions:
      if str(action_instance) in str(node):
        action_duration[node] = float(duration)
        action_variance[node] = 2
    ## find node string in plan actions
    # for action in plan.timed_actions:
        # if action in :
        #      print(action)
            ## find the nominal value of the node

    # nominal_value[node] = 
print("----------")
EPS = 0.1
import numpy as np
end_values = []
for _ in range(1):
  model = pywraplp.Solver.CreateSolver('GLOP')
  action_duration = {}
  action_variance = {}
  for node in constraints:
      time_vars[node] = model.NumVar(0, 1000, f"time_{node}")
      for (start, action_instance, duration) in plan.timed_actions:
        if str(action_instance) in str(node):
          action_duration[node] = float(duration)
          action_variance[node] = 2

  for node_A, edges in constraints.items():
      # print(node_A)
      if node_A in action_duration:
        # print(f"action_duration: {action_duration[node_A]}")
        nominal = action_duration[node_A]
        alter = nominal
        alter = max(np.random.normal(nominal, 0.1), 0.1)
        alter = nominal - 0.1
        # print(f'alter: {alter}')
        # alter = max(np.random.uniform(action_duration[node_A], 1),2)
      for lower_bound, upper_bound, node_B in edges:
          # print(node_B)
          if node_A not in action_duration:
            if lower_bound is not None:
                # print(float(lower_bound))
                model.Add(time_vars[node_B] - time_vars[node_A] >= float(lower_bound))
            if upper_bound is not None:
                # print(float(upper_bound))
                model.Add(time_vars[node_B] - time_vars[node_A] <= float(upper_bound))
          else:
            if lower_bound is not None:
                if float(lower_bound) < 0.01:
                  # print(float(lower_bound))
                  model.Add(time_vars[node_B] - time_vars[node_A] >= float(lower_bound))
                else:
                  # print(float(alter))
                  model.Add(time_vars[node_B] - time_vars[node_A] >= float(alter))
            if upper_bound is not None:
                if float(upper_bound) < 0.01:
                  # print(float(upper_bound))
                  model.Add(time_vars[node_B] - time_vars[node_A] <= float(upper_bound))
                else:
                  # print(float(alter))
                  model.Add(time_vars[node_B] - time_vars[node_A] <= float(alter))
          
      print("--------------------------------")
  model.Minimize(time_vars[end_plan])
  status = model.Solve()
  # Verifica lo stato della soluzione e stampa i risultati
  if status == pywraplp.Solver.OPTIMAL:
      for variable in time_vars:
          print(f"{variable}: {time_vars[variable].solution_value()}")
  else:
      print('Nessuna soluzione ottimale trovata.')

  end_values.append(time_vars[end_plan].solution_value())

print(f"Mean: {np.mean(end_values)}")
print(f"Std: {np.std(end_values)}")