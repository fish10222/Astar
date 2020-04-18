import math

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]
"""
def is_illegal(child_loc, my_map):
    # Check out of bound moves
    retVal = False
    if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] > len(my_map) - 1 or child_loc[1] > len(my_map[0]) - 1 or my_map[child_loc[0]][child_loc[1]]:
        retVal = True
    return retVal

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
"""
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
"""
    Lookup table for OSF
    {
        ∆h: ∆f
    }
"""
lookup_table = {
    -1:0,
    0:1,
    1:2,
}
def OSF (fval, curr_node, h_values, my_map, constraint_table, moves, num_of_agents, closed_list):
    """
        f_next =  fval + min(∆f)
        returns ([list of locations], f_next)
    """
    f_next = []
    for i in range(num_of_agents):
        f_next.append(math.inf)

    newlocs = []
    #curr_time = curr_node['timestep']
    #curr_hval = curr_node['h_val']
    current_loc = curr_node['locs']
    print("move")
    for p in list(moves):
        next_loc = [move(curr_node['locs'][i], p[i]) for i in range(num_of_agents)]
        # print('OSF NEXT LOC')
        # print(next_loc)
        if any_out_of_map(next_loc, my_map):
            continue

        out_of_bound = [my_map[next_loc[i][0]][next_loc[i][1]] for i in range(num_of_agents)]
        if True in out_of_bound:
            continue

        # check if externally constrained, then continue
        if any_ext_constrained(curr_node['locs'], next_loc, curr_node['timestep']+1, constraint_table):
            continue
        ####
        # check if any move results in collision, then continue
        if any_conflicted(curr_node['locs'], next_loc):
            continue
        ####
        ####
        if (tuple(next_loc), curr_node['timestep']+1) in closed_list:
            continue
        ####
        f_next_node = []
        for i in range(num_of_agents):
            curr_hval = h_values[i][current_loc[i]]
            print(curr_hval)
            next_hval = h_values[i][next_loc[i]]
            print(next_hval)
            delta_h = next_hval - curr_hval
            delta_f = lookup_table[delta_h]
            f_next_node.append(curr_hval + curr_node['g_val'] + delta_f)
        if f_next_node != fval and f_next_node > fval:
                f_next= min(f_next, f_next_node)   #set f_next to samllest f
        print("PASSED")
        print(next_loc)
        print(f_next_node)
        if f_next_node == fval:         #if next node matches the f_value, we put in list
            newlocs.append(next_loc)
        
    #if math.inf in f_next:
    if len(newlocs) <= 0:
        for f in range(len(f_next)):
            f_next[f] = -1

    answers = (newlocs, f_next)
    print('OSF ANSWER')
    print(answers)
    #returns ((tuple of valid moves), fnext)
    return answers