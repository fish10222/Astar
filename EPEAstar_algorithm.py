import heapq
import copy
import time
from itertools import product
from functools import total_ordering
from OSF import OSF

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
    heapq.heappush(open_list, KeyDict((node['g_val'] + node['h_val'], node['h_val'], node['locs']) , node))



def pop_node(open_list):
    curr = heapq.heappop(open_list)
    return curr.dct

def build_ext_constraints_tbl(constraints):
    constraintTable = dict()
    for constraint in constraints:
        if constraint['timestep'] not in constraintTable:
            constraintTable[constraint['timestep']] = set()

        if len(constraint['locs']) == 1:
            constraintTable[constraint['timestep']].add(constraint['locs'][0])

        # Edge constraint
        elif len(constraint['locs']) == 2:
            constraintTable[constraint['timestep']].add(constraint['locs'][0] + constraint['locs'][1])

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
            retVal[j].append(curr['locs'][j])
        curr = curr['parent']

    for path in retVal:
        path.reverse()

    print(retVal)
    return retVal

def epea_star(my_map, start_locs, goal_locs, h_values, ext_constraints):
    # EPEA*
    num_of_agents = len(goal_locs)
    constraintTable = build_ext_constraints_tbl(ext_constraints)
    open_list = []
    closed_list = dict()
    h_value = 0
    h_value_seperate = list()
    for i in range(0, num_of_agents):
        h_value += h_values[i][start_locs[i]]
        h_value_seperate.append(h_values[i][start_locs[i]])
    f_value = h_value_seperate #g(n) + h(n), since g(n) = 0
    f_next = 0 #placeholder for F-next
    # Fix goal constraints
    constraint_timesteps = constraintTable.keys()
    earliest_goal_timestep = max(constraint_timesteps) if constraint_timesteps else 0

    root = {'locs': tuple(start_locs),
            'g_val': 0,
            'h_val': h_value,
            'h_val_sep': tuple(h_value_seperate),
            'f_val': f_value,
            'parent': None,
            'timestep': 0}
    push_node(open_list, root)
    closed_list[(root['locs'], root['timestep'])] = root


    while len(open_list) > 0:
        curr = pop_node(open_list)
        print('POP')
        print(curr)
        time.sleep(1)
        if curr['locs'] == tuple(goal_locs) and curr['timestep'] >= earliest_goal_timestep:
            return get_path(curr, num_of_agents)


        #Set N and Fnext
        f_value = curr['f_val']
        print('FVAL')
        print(f_value)
        perm = product(list(range(5)), repeat=num_of_agents) #list of all possible moves
        osf_ans = OSF(f_value, curr, h_values, my_map, constraintTable, perm, num_of_agents, closed_list)
        N = osf_ans[0]
        f_next = osf_ans[1]
        for nc in N:
            child_locs = nc
            #get sum of hval from each agent
            h_value = 0
            h_value_seperate = []
            for i in range(num_of_agents):
                h_value = h_value + h_values[i][child_locs[i]]
                h_value_seperate.append(h_values[i][start_locs[i]])

            child = {'locs': tuple(child_locs),
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_value,
                    'h_val_sep': tuple(h_value_seperate),
                    'f_val': f_value,
                    'parent': curr,
                    'timestep': curr['timestep'] + 1}
            if (child['locs'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['locs'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['locs'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                push_node(open_list, child)
        if -1 in f_next:  #if f-next is infinity, put it in closed
            closed_list[(curr['locs'], curr['timestep'])] = child
        else:
            curr['f_val'] = f_next
            push_node(open_list, curr)
    return None