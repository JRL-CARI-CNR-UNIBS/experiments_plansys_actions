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
from ortools.linear_solver import pywraplp
from unified_planning.io.utils import parse_string
# from unified_planning.model.expression import 
domain_path = "/home/kalman/projects/turtlebot_ws/src/experiments_plansys_actions/pddl/tamer/domain.pddl"
problem_path = "/home/kalman/projects/turtlebot_ws/src/experiments_plansys_actions/pddl/tamer/problem.pddl"

def get_str_solution():
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
  return example_plan

reader = PDDLReader()
try:
  parsed_problem = reader.parse_problem(domain_path, problem_path)
except Exception as e:
  raise e
solver = "tamer"
with OneshotPlanner(name=solver) as planner:
    result = planner.solve(parsed_problem)

plan = result.plan
example_plan = get_str_solution()
plan = reader.parse_plan_string(problem=parsed_problem, plan_str=example_plan)

stn_plan = plan.convert_to(PlanKind.STN_PLAN,parsed_problem)


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

for _ in range(100):
    
  model = pywraplp.Solver.CreateSolver('GLOP')

  for node in constraints:
      time_vars[node] = (model.NumVar(0, 1000, f"time_{node}"),5)
      action, variance, nominal_value = None, None, None
      print(str(node))
      for (start, action_instance, duration) in plan.timed_actions:
        print(str(action_instance))
        print(str(action_instance) in str(node))
        if str(action_instance) in str(node):
          action = str(action_instance)
          variance = 2
          nominal_value = duration
      # if action is None or variance is None or nominal_value is None:
      #   raise Exception("action or variance or nominal_value not found")


  for node_A, edges in constraints.items():

      for lower_bound, upper_bound, node_B in edges:
          
          
          if lower_bound is not None:
              print(float(lower_bound))
              if float(lower_bound) < 0.01:
                model
              else:
                model.Add(time_vars[node_B][0] - time_vars[node_A][0] >= float(lower_bound))
          if upper_bound is not None:
              print(float(upper_bound))
              model.Add(time_vars[node_B][0] - time_vars[node_A][0] <= float(upper_bound))
      print("--------------------------------")
  model.Minimize(time_vars[end_plan][0])

