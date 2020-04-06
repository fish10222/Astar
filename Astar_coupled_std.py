import heapq
from itertools import product

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['locs'] , node))

def pop_node(open_list):

    _, _, _, curr = heapq.heappop(open_list)
    return curr

def build_ext_constraints_tbl(constraints): 
    table = {}
    for c in constraints:
        if not c['timestep'] in table:
            table[c['timestep']] = []
        table[c['timestep']].append(c['loc'])
    return table


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

def any_ext_constrained(prev_locs, curr_locs, curr_time, constraint_table):
    # External constraints check
    prev_locs = list(prev_locs)
    curr_locs = list(curr_locs)
    num_of_agents = len(curr_locs)
    for a in range(num_of_agents):
        if curr_time in constraint_table:
            if [curr_locs[a]] in constraint_table[curr_time]:
                return True
            elif [curr_locs[a], prev_locs[a]] in constraint_table[curr_time]:
                return True
    return False

def is_illegal(child_loc, my_map):
    # Check out of bound moves
    retVal = False
    if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] > len(my_map) - 1 or child_loc[1] > len(my_map[0]) - 1 or my_map[child_loc[0]][child_loc[1]]:
        retVal = True
    return retVal

def any_out_of_map(child_locs, my_map):
    child_locs = list(child_locs)
    h = len(my_map)
    w = len(my_map[0])
    for loc in child_locs:
        if loc[0] < 0 or loc[1] < 0 or loc[0] >= h or loc[1] >= w:
            return True
    return False

def any_conflicted(prev_locs, curr_locs): 
    # Check if any move is conflicted (agents moving into each other)
    ## agents running into each other at a vertex

    if len(curr_locs) != len(set(curr_locs)):
        return True
    ## 2 agents running into each other on an edge
    curr_locs = list(curr_locs)
    prev_locs = list(prev_locs)
    for i in range(len(curr_locs)):
        if curr_locs[i] in prev_locs and curr_locs[i] != prev_locs[i]:
            if curr_locs[prev_locs.index(curr_locs[i])] == prev_locs[i]:
                return True

    return False

def get_path(goal_node): 
    paths = [ [] for i in goal_node['locs'] ]

    paths_len = goal_node['finished_cost'][:]
    if set(paths_len) == -1:
        return goal_node['locs']
    curr = goal_node
    while curr is not None:
        t = curr['timestep']
        for j in range(len(curr['locs'])):
            if t <= paths_len[j]:
                paths[j].append(curr['locs'][j])
        curr = curr['parent']

    for path in paths:
        path.reverse()

    
    return paths

def a_star_coupled(my_map, start_locs, goal_locs, h_values, ext_constraints):  ###### TODO: single agent version and determine no solution
    # A* for multiple agents
    num_of_agents = len(goal_locs)
    count = 0

    open_list = []
    closed_list = dict()
    
    earliest_goal_timestep = 0
    ### build constraint table
    constraint_table = build_ext_constraints_tbl(ext_constraints)

    ### initial h_value
    h_value = 0
    for i in range(num_of_agents):
        h_value = h_value + h_values[i][start_locs[i]]
    root = {'locs': tuple(start_locs), 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep' : 0, 'finished_cost' : [-1 for i in range(num_of_agents)]}
    ### if an agent spawns at its goal, set cost 0
    for i in range(num_of_agents):
        if root['locs'][i] == goal_locs[i]:
            root['finished_cost'][i] = 0
    #if num_of_agents = 1:
    #    root['locs'] = root['locs'][0]
    #else:
    #    root['locs'] = tuple(root['locs'])
    push_node(open_list, root)
    closed_list[(root['locs'], 0)] = root ##use tuple for locs
    while len(open_list) > 0:
        
        curr = pop_node(open_list)
        
        count+=1
        ### goal condition and get path
        if curr['locs'] == tuple(goal_locs) or curr['locs'] == goal_locs[0]:
            return get_path(curr)

        #######################
        perm = product(list(range(5)), repeat=num_of_agents)
        
        for p in list(perm):
            child_locs = [move(curr['locs'][i], p[i]) for i in range(num_of_agents)]

            if any_out_of_map(child_locs, my_map):
                continue
            out_of_bound = [my_map[child_locs[i][0]][child_locs[i][1]] for i in range(num_of_agents)]
        
            if True in out_of_bound:
                continue


            # check if externally constrained, then continue
            if any_ext_constrained(curr['locs'], child_locs, curr['timestep']+1, constraint_table):
                continue
            ####

            # check if any move results in collision, then continue
            if any_conflicted(curr['locs'], child_locs):
                
                continue
            ####

            #### update h_value
            h_value = 0

            for i in range(num_of_agents):
                h_value = h_value + h_values[i][child_locs[i]]
            #####

            child = {'locs': tuple(child_locs),
                    'g_val': curr['g_val'],
                    'h_val': h_value,
                    'parent': curr,
                    'timestep': curr['timestep']+1,
                    'finished_cost': curr['finished_cost'][:]}
            ##### check if any agent JUST REACHES its goal, if yes, then update finished cost of that agent
            ############## also, if any agent LEAVES its goal, then update finished cost of that agent 
            ############## we update g_values accordingly by the way
            for i in range(num_of_agents):
                
                    
                if child['locs'][i] != goal_locs[i] and child['finished_cost'][i] != -1:                    
                    # add the missed cost to g_val
                    child['g_val'] = child['g_val'] + child['timestep'] - child['finished_cost'][i]                   
                    child['finished_cost'][i] = -1

                else:
                    if child['locs'][i] == goal_locs[i]:
                        if child['finished_cost'][i] == -1:
                            child['finished_cost'][i] = child['timestep']
                        
                    if curr['locs'][i] == goal_locs[i]:
                        continue
                    child['g_val'] = child['g_val'] + 1

            #####
            if (child['locs'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['locs'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['locs'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['locs'], child['timestep'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions

