import time as timer
from single_agent_planner import compute_heuristics, get_sum_of_cost, a_star, get_location

from Astar_coupled import a_star_coupled

def detect_collisions(paths):
    # Return a list of first collisions between all robot pairs.

    collisions = list()
    for i in range(len(paths) - 1):
        for j in range(i + 1, len(paths)):
            collision = detect_collision(paths[i], paths[j])
            if collision:
                collision['a1'] = i
                collision['a2'] = j
                collisions.append(collision)
    return collisions

def detect_collision(path1, path2):
    # Return the first collision that occurs between two robot paths (or None if there is no collision)

    for i in range(max(len(path1), len(path2))):
        loc1 = get_location(path1, i)
        loc2 = get_location(path2, i)
        if loc1 == loc2:
            return {'loc': [loc1], 'timestep': i}
        if i > 0 and loc1 == get_location(path2, i-1) and loc2 == get_location(path1, i-1):
            return {'loc': [loc1, loc2], 'timestep': i}
    return None

def merge_group(group_list, g1, g2, agentList):
    group_list[g1] = group_list[g1] + group_list[g2]
    for i in group_list[g1]:
        agentList[i] = g1
    return group_list[g1]
    
class AstarSolver(object):
    """A planner that plans for each robot independently."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()     
        
        result = []
        result = a_star_coupled(self.my_map, self.starts, self.goals, self.heuristics, [])

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))

        return result
