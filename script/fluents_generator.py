import random
from itertools import permutations, product

# Waypoints list
waypoints = [
    "start_wp_robot1", "start_wp_robot2",
    "hall", "entrance",
    "clean1", "clean2", "clean3", "sofa"
]

# Robots list
robots = ["robot1", "robot2"]

# Min-Max duration
min_duration = 2
max_duration = 15

# Generates all simple arrangements in groups of 2 waypoints
waypoint_perms = permutations(waypoints, 2)

# Combine robots with waypoint arrangements
robot_waypoint_combinations = product(robots, waypoint_perms)

# Function for generating duration fluents with random durations
def generate_fluents(combinations, min_dur, max_dur):
    fluents = []
    for robot, (wp1, wp2) in combinations:
        duration = random.randint(min_dur, max_dur)
        fluents.append(f"(= (duration_move {robot} {wp1} {wp2}) {duration})")
    return fluents

# Generate fluents
fluent_statements = generate_fluents(robot_waypoint_combinations, min_duration, max_duration)

# Print
for statement in fluent_statements:
    print(statement)
