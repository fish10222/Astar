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
    # print((node['g_val'] + node['h_val'], node['h_val'], node['loc']))
    assignedNodeList = [node['loc'][i] for i in range(len(node['loc']) - node['unassigned'])]
    heapq.heappush(open_list, KeyDict((node['g_val'] + node['h_val'], node['h_val'], assignedNodeList), node))


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

def get_path(goal_node, agentCount):
    retVal = [ [] for i in range(agentCount) ]

    curr = goal_node
    while curr is not None:
        for j in range(agentCount):
            retVal[j].append(curr['loc'][j])
        curr = curr['parent']

    for path in retVal:
        i = 0
        for j in range(1, len(path)):
            if path[0] == path[j]:
                i += 1
            else:
                break
        
        path.reverse()
        if i != 0: del path[-i:]

    return retVal

    
def a_star_coupled(my_map, start_locs, goal_locs, h_values, ext_constraints):
    # A* + OD for multiple agents
    # print("MA_CBS triggered")

    agentCount = len(start_locs)

    constraintTable = build_ext_constraints_tbl(ext_constraints)
    open_list = []
    closed_list = dict()
    
    h_value = 0
    for i in range(0, agentCount):
        h_value += h_values[i][start_locs[i]]

    # Fix goal constraints
    constraint_timesteps = constraintTable.keys()
    earliest_goal_timestep = max(constraint_timesteps) if constraint_timesteps else 0

    root = {'loc': tuple(start_locs), 
            'g_val': 0,
            'h_val': h_value, 
            'parent': None, 
            'unassigned': 0,
            'timestep': 0}
    push_node(open_list, root)

    closed_list[(root['loc' ], root['timestep'])] = root

    while len(open_list) > 0:
        curr = pop_node(open_list)

        if curr['loc'] == tuple(goal_locs) and curr['timestep'] >= earliest_goal_timestep:
            return get_path(curr, agentCount)

        if curr['unassigned'] == 0:
            # Intermediate Node
            for dir in range(5):
                child_loc = move(curr['loc'][0], dir)
                
                if is_illegal(child_loc, my_map) or is_ext_constrained(curr['loc'], child_loc, curr['timestep'] + 1, constraintTable):
                    continue
                
                new_loc = tuple(child_loc if i == 0 else None for i in range(agentCount))

                child = {'loc': new_loc,
                        'g_val': curr['g_val'] + 1, 
                        'h_val': h_values[0][child_loc],
                        'parent': curr,
                        'unassigned': agentCount - 1,
                        'timestep': curr['timestep'] + 1}

                if (child['loc'], child['timestep']) in closed_list:
                    existing_node = closed_list[(child['loc'], child['timestep'])]
                    if compare_nodes(child, existing_node):
                        closed_list[(child['loc'], child['timestep'])] = child
                        push_node(open_list, child)
                else:
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)

        
        else:
            agent = agentCount - curr['unassigned']
            # isLastAgent = (curr['unassigned'] == 1)

            for dir in range(5):
                child_loc = move(curr['parent']['loc'][agent], dir)

                if is_illegal(child_loc, my_map) or is_ext_constrained(curr['parent']['loc'][agent], child_loc, curr['timestep'], constraintTable):
                    continue

                if is_conflicted(curr['loc'], curr['parent']['loc'], agent, child_loc):
                    continue
                
                new_loc = list(curr['loc'])
                new_loc[agent] = child_loc

                child = {'loc': tuple(new_loc), 
                        'g_val': (curr['g_val'] + 1), #if isLastAgent else curr['g_val'], 
                        'h_val': curr['h_val'] + h_values[agent][child_loc],
                        'parent': curr['parent'],
                        'unassigned': curr['unassigned'] - 1,
                        'timestep': curr['timestep']}
                        
                if (child['loc'], child['timestep']) in closed_list:
                    existing_node = closed_list[(child['loc'], child['timestep'])]
                    if compare_nodes(child, existing_node):
                        closed_list[(child['loc'], child['timestep'])] = child
                        push_node(open_list, child)
                else:
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
                

    return None