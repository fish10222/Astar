import heapq
import copy
from functools import total_ordering

@total_ordering
class KeyDict(object):
    # Need this since you cant compare dict()
    def __init__(self, key, dct):
        self.key = key
        self.dct = dct

    def __lt__(self, other):
        return self.key < other.key

    def __eq__(self, other):
        return self.key == other.key

    def __repr__(self):
        return '{0.__class__.__name__}(key={0.key}, dct={0.dct})'.format(self)


def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]

def push_node(open_list, node):
    heapq.heappush(open_list, KeyDict((node['g_val'] + node['h_val'], node['h_val'], node['loc']) , node))


def pop_node(open_list):
    curr = heapq.heappop(open_list)
    return curr.dct

def build_ext_constraints_tbl(constraints):
    constraintTable = dict()
    for constraint in constraints:
        if constraint['timestep'] not in constraintTable:
            constraintTable[constraint['timestep']] = set()
        
        if len(constraint['loc']) == 1:
            constraintTable[constraint['timestep']].add(constraint['loc'][0])
        
        # Edge constraint
        elif len(constraint['loc']) == 2:
            constraintTable[constraint['timestep']].add(constraint['loc'][0] + constraint['loc'][1])
    
    return constraintTable

def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

def is_ext_constrained(curr_loc, next_loc, next_time, constraint_table):
    # External constraints check
    if not constraint_table:
        return False

    if next_time not in constraint_table:
        return False
    
    edgeConstraint = curr_loc + next_loc

    if (next_loc in constraint_table[next_time]) or (edgeConstraint in constraint_table[next_time]):
        return True
    return False

def is_illegal(child_loc, my_map):
    # Check out of bound moves
    retVal = False
    if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] > len(my_map) - 1 or child_loc[1] > len(my_map[0]) - 1 or my_map[child_loc[0]][child_loc[1]]:
        retVal = True
    return retVal

def is_conflicted(loc, parent_loc, agent, child_loc):
    # Check if a move is conflicted (agents moving into each other)
    for i in range(len(loc)):
        if not loc[i]:
            break

        if child_loc == loc[i]:
            return True
        if loc[i] == parent_loc[agent] and child_loc == parent_loc[i]:
            return True

    return False

def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path

def epea_star(my_map, start_loc, goal_loc, h_values, agent, ext_constraints):
    # EPEA* 

    constraintTable = build_ext_constraints_tbl(ext_constraints)
    open_list = []
    closed_list = dict()
    
    h_value = 0
    h_value = h_values[start_loc]
    f_value = h_value #current F-value
    f_next = 0 #placeholder for F-next
    # Fix goal constraints
    constraint_timesteps = constraintTable.keys()
    earliest_goal_timestep = max(constraint_timesteps) if constraint_timesteps else 0

    root = {'loc': start_loc, 
            'g_val': 0,
            'h_val': h_value, 
            'parent': None, 
            'timestep': 0}
    push_node(open_list, root)

    closed_list[(root['loc' ], root['timestep'])] = root

    while len(open_list) > 0:
        curr = pop_node(open_list)

        if curr['loc'] == goal_loc and curr['timestep'] >= earliest_goal_timestep:
            return get_path(curr)

        #Set N and Fnext

        for nc in N:
            child_loc = move(curr['loc'], nc)

            if is_illegal(child_loc, my_map) or is_ext_constrained(curr['parent']['loc'][agent], child_loc, curr['timestep'], constraintTable):
                 continue

            if is_conflicted(curr['loc'], curr['parent']['loc'], agent, child_loc):
                continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr}
            if (child['loc']) in closed_list:
                existing_node = closed_list[(child['loc'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'])] = child
                    push_node(open_list, child)
        if f_next < 0:  #if f-next is infinity, put it in closed
            closed_list[(child['loc'])] = child
        else:
            f_value = f_next 
            push_node(open_list, curr)
                

    return None