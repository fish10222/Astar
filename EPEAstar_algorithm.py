import heapq
import copy
import time
import math
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
    heapq.heappush(open_list, KeyDict((sum(node['f_val']), node['h_val'], node['locs']) , node))
    #heapq.heappush(open_list, KeyDict((node['g_val'] + node['h_val'], node['h_val'], node['locs']) , node))



def pop_node(open_list):
    curr = heapq.heappop(open_list)
    return curr.dct

def build_ext_constraints_tbl(constraints):
    table = {}
    for c in constraints:
        if not c['timestep'] in table:
            table[c['timestep']] = []
        for a in c['agents']:
            table[c['timestep']].append({'agent':a , 'loc':c['loc']})
    return table

def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

def get_path(goal_node, agentCount):
    retVal = [ [] for i in range(agentCount) ]

    curr = goal_node
    while curr is not None:
        for j in range(agentCount):
            retVal[j].append(curr['locs'][j])
        curr = curr['parent']

    for path in retVal:
        path.reverse()


    return retVal

def epea_star(my_map, start_locs, goal_locs, h_values, ext_constraints):
    # EPEA*
    num_of_agents = len(goal_locs)
    constraintTable = build_ext_constraints_tbl(ext_constraints)
    open_list = []
    closed_list = dict()
    h_value = 0
    f_value_seperate = list()
    g_value_seperate = list()
    for i in range(0, num_of_agents):
        h_value += h_values[i][start_locs[i]]
        f_value_seperate.append(h_values[i][start_locs[i]])
        g_value_seperate.append(0)
    f_value = f_value_seperate #g(n) + h(n), since g(n) = 0
    f_next = 0 #placeholder for F-next
    # Fix goal constraints
    constraint_timesteps = constraintTable.keys()
    earliest_goal_timestep = max(constraint_timesteps) if constraint_timesteps else 0

    root = {'locs': tuple(start_locs),
            'g_val': 0,
            'g_val_sep': g_value_seperate,
            'h_val': h_value,
            'f_val': f_value,
            'parent': None,
            'timestep': 0}
    ### if an agent spawns at its goal, set cost 0
    # for i in range(num_of_agents):
    #     if root['locs'][i] == goal_locs[i]:
    #         root['finished_cost'][i] = 0
    push_node(open_list, root)
    closed_list[(root['locs'], root['timestep'])] = root


    while len(open_list) > 0:
        curr = pop_node(open_list)

        if curr['locs'] == tuple(goal_locs) and curr['timestep'] >= earliest_goal_timestep:
            return get_path(curr, num_of_agents)


        #Set N and Fnext
        f_value = curr['f_val']
        perm = product(list(range(5)), repeat=num_of_agents) #list of all possible moves
        osf_ans = OSF(f_value, curr, h_values, my_map, goal_locs, constraintTable, perm, num_of_agents, closed_list)
        N = osf_ans[0]
        f_next = osf_ans[1]
        for nc in N:
            child_locs = nc
            #get sum of hval from each agent
            h_value = 0
            for i in range(num_of_agents):
                h_value = h_value + h_values[i][child_locs[i]]

            child = {'locs': tuple(child_locs),
                    'g_val': copy.deepcopy(curr['g_val']),
                    'g_val_sep': copy.deepcopy(curr['g_val_sep']),
                    'h_val': h_value,
                    'f_val': f_value,
                    'parent': curr,
                    'timestep': curr['timestep'] + 1}

            for i in range(num_of_agents):
                if curr['locs'][i] == child['locs'][i] == goal_locs[i]:
                    child['g_val_sep'][i] = child['g_val_sep'][i]
                else:
                    child['g_val_sep'][i] = child['g_val_sep'][i] + 1
                    child['g_val'] = child['g_val'] + 1

            if (child['locs'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['locs'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['locs'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                push_node(open_list, child)
        if -1 in f_next:  #if f-next is infinity, put it in closed
            closed_list[(curr['locs'], curr['timestep'])] = curr
        else:
            curr['f_val'] = f_next
            push_node(open_list, curr)
    return None