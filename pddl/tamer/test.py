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
print(plan)
print(type(plan))
for action in plan.timed_actions:
    print(action[1].action.name)
    print(action[1].actual_parameters)
    print(action[0])
fds
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

for _ in range(100):
    
  model = pywraplp.Solver.CreateSolver('GLOP')

  for node in constraints:
      time_vars[node] = model.NumVar(0, 1000, f"time_{node}")
      variances[node] = 2
      ## find node string in plan actions
      print(node.action_instance.action.name)
      print(node.action_instance.actual_parameters)
      # for action in plan.timed_actions:
          # if action in :
          #      print(action)
              ## find the nominal value of the node

      # nominal_value[node] = 
  print("----------")
  for node_A, edges in constraints.items():
      print(node_A)
      for lower_bound, upper_bound, node_B in edges:
          
          
          if lower_bound is not None:
              print(float(lower_bound))
              if float(lower_bound) < 0.01:
                model
              else:
                model.Add(time_vars[node_B] - time_vars[node_A] >= float(lower_bound))
          if upper_bound is not None:
              print(float(upper_bound))
              model.Add(time_vars[node_B] - time_vars[node_A] <= float(upper_bound))
      print("--------------------------------")
  model.Minimize(time_vars[end_plan])

# #Risolvi il modello
# status = model.Solve()

# # Verifica lo stato della soluzione e stampa i risultati
# if status == pywraplp.Solver.OPTIMAL:
#     for variable in time_vars:
#         print(f"{variable}: {time_vars[variable].solution_value()}")
# else:
#     print('Nessuna soluzione ottimale trovata.')


# for _ in range(1000):
#     constraints = stn_plan.get_constraints()

#     time_vars = {}

#     start_plan = STNPlanNode(TimepointKind.GLOBAL_START)
#     end_plan = STNPlanNode(TimepointKind.GLOBAL_END)
#     if start_plan not in constraints:
#         constraints[start_plan] = []
#     if end_plan not in constraints:
#         constraints[end_plan] = []

#     # Crea variabili per ciascun nodo
#     for node in constraints:
#         time_vars[node] = model.NumVar(0, 1000, f"time_{node}")

#     for node_A, edges in constraints.items():
#         for lower_bound, upper_bound, node_B in edges:
#             if lower_bound is not None:
#                 print(float(lower_bound))
#                 model.Add(time_vars[node_B] - time_vars[node_A] >= float(lower_bound))
#             if upper_bound is not None:
#                 print(float(upper_bound))
#                 model.Add(time_vars[node_B] - time_vars[node_A] <= float(upper_bound))
#         print("--------------------------------")
#     model.Minimize(time_vars[end_plan])

#     #Risolvi il modello
#     status = model.Solve()

# #### 
# from typing import Dict, List, Optional, Set, Tuple
# from unified_planning.model import (
#     InstantaneousAction,
#     Timing,
#     DurativeAction,
#     Problem,
#     FNode,
#     TimepointKind,
#     Effect,
#     TimeInterval,
#     Timepoint,
#     SimulatedEffect,
# )
# from itertools import chain
# import unified_planning.plans as plans
# from unified_planning.plans.time_triggered_plan import TimeTriggeredPlan,  get_partial_order_plan

# from unified_planning.plans import SequentialPlan, PartialOrderPlan
# from unified_planning.model import Problem
# from fractions import Fraction
# from typing import Tuple

# from collections import OrderedDict
# from unified_planning.plans import SequentialPlan, PartialOrderPlan, ActionInstance
# from unified_planning.model import DurativeAction, InstantaneousAction, Problem
# from fractions import Fraction
# from typing import Tuple, List, Dict, Set

# # print(get_partial_order_plan(plan, parsed_problem))

# # def extract_action_timings(
# #     action: DurativeAction,
# #     start: Fraction,
# #     duration: Fraction,
# #     epsilon: Fraction = Fraction(0),
# # ) -> Set[Fraction]:
# #     """
# #     Estrae tutti i tempi interessanti dell'azione: dove inizia/finisce una condizione,
# #     dove si verifica un effetto o un effetto simulato.
# #     """
# #     timings: Set[Fraction] = set()
    
# #     def absolute_time(relative_time):
# #         return start + (relative_time.delay if relative_time.is_from_start() else duration + relative_time.delay)

# #     timings.update(absolute_time(t) for t in action.effects.keys())
# #     timings.update(absolute_time(t) for t in action.simulated_effects.keys())

# #     for interval in action.conditions.keys():
# #         lower_increment = epsilon if interval.is_left_open() else Fraction(0)
# #         upper_increment = -epsilon if interval.is_right_open() else Fraction(0)
# #         timings.add(absolute_time(interval.lower) + lower_increment)
# #         timings.add(absolute_time(interval.upper) + upper_increment)

# #     return timings


# # def convert_plan_to_seq_and_partial_order(
# #     time_triggered_plan: plans.TimeTriggeredPlan,
# #     problem: Problem,
# # ) -> Tuple[SequentialPlan, PartialOrderPlan]:
# #     """
# #     Questa funzione accetta un `TimeTriggeredPlan` e restituisce un `SequentialPlan` e un `PartialOrderPlan`.
    
# #     :param time_triggered_plan: Il piano di ingresso di tipo `TimeTriggeredPlan`.
# #     :param problem: Il problema associato a questo piano.
# #     :return: Una tupla contenente `SequentialPlan` e `PartialOrderPlan`.
# #     """
# #     events: Dict[Fraction, List[ActionInstance]] = {}
# #     event_creating_ais = {}

# #     # Costruzione della lista di eventi e mappatura delle istanze d'azione
# #     for start, ai, duration in time_triggered_plan.timed_actions:
# #         if duration is None:
# #             events.setdefault(start, []).append(ai)
# #             event_creating_ais[ai] = (ai, Fraction(0))
# #         else:
# #             action_timings = extract_action_timings(ai.action, start, duration)
# #             for i, timing in enumerate(sorted(action_timings)):
# #                 # Conversione dei parametri in OrderedDict
# #                 parameters_dict = OrderedDict((p.name, p.type) for p in ai.action.parameters)
# #                 inst_action = InstantaneousAction(f"{ai.action.name}_{i}", parameters_dict)
# #                 inst_ai = ActionInstance(inst_action, ai.actual_parameters)
# #                 events.setdefault(timing, []).append(inst_ai)
# #                 relative_timing = timing - start
# #                 event_creating_ais[inst_ai] = (ai, relative_timing)

# #     sorted_events = sorted(events.items(), key=lambda x: x[0])

# #     # Creazione di un SequentialPlan
# #     list_act = [action_instance for _, actions in sorted_events for action_instance in actions]
# #     seq_plan = SequentialPlan(list_act)

# #     # Conversione a PartialOrderPlan
# #     # partial_order_plan = seq_plan.convert_to(PartialOrderPlan, problem)
    
# #     return seq_plan #, partial_order_plan


# # seq_plan = convert_plan_to_seq_and_partial_order(plan, parsed_problem)
# # partial_order_plan = seq_plan.convert_to(plans.PlanKind.PARTIAL_ORDER_PLAN, parsed_problem)
# # print(seq_plan)
# # print(partial_order_plan)