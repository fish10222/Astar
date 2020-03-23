import heapq

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr

def build_ext_constraints_tbl(constraints):
    if not constraints: 
        return None
    
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
    if next_time not in constraint_table:
        return False
    
    edgeConstraint = curr_loc + next_loc

    if (next_loc in constraint_table[next_time]) or (edgeConstraint in constraint_table[next_time]):
        return True
    return False

def is_illegal(child_loc, my_map):
    retVal = False
    if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] > len(my_map) - 1 or child_loc[1] > len(my_map[0]) - 1 or my_map[child_loc[0]][child_loc[1]]:
        retVal = True
    return retVal

def is_conflicted(loc0, parent0, loc1, parent1):
    if loc0 == loc1:
        return True
    if loc0 == parent1 and loc1 == parent0:
        return True
    return False

def get_path(goal_node):
    path0 = []
    path1 = []
    curr = goal_node
    while curr is not None:
        path0.append(curr['loc0'])
        path1.append(curr['loc1'])
        curr = curr['parent']
    path0.reverse()
    path1.reverse()
    return [path0, path1]

def a_star_coupled(my_map, start_locs, goal_locs, h_values, ext_constraints):
    # A* + OD for 2 agents ONLY
    # TODO: Heuristics

    constraintTable = build_ext_constraints_tbl(ext_constraints)
    open_list = []
    closed_list = dict()

    root = {'loc0': start_locs[0], 'loc1': start_locs[1], 'g_val': 0,
            'h_val': sum(h_values), 'parent': None, 'timestep': 0}
    push_node(open_list, root)

    closed_list[(root['loc0'], root['loc1'], root['timestep'])] = root

    while len(open_list) > 0:
        curr = pop_node(open_list)

        if curr['loc0'] == goal_locs[0] and curr['loc1'] == goal_locs[1]:
            return get_path(curr)

        if curr['loc1']:
            # Intermediate Node
            for dir in range(5):
                child_loc = move(curr['loc0'], dir)
                
                if is_illegal(child_loc, my_map) or is_ext_constrained(curr['loc0'], child_loc, curr['timestep'] + 1, constraintTable):
                    continue

                child = {'loc0': child_loc, 
                        'loc1': None,
                        'g_val': curr['g_val'] + 1, 
                        'h_val': h_values['child_loc'],
                        'parent': curr,
                        'timestep': curr['timestep'] + 1}

                # Only push standard nodes into closed list
                push_node(open_list, child)
        else:
            # Standard Node
            for dir in range(5):
                child_loc = move(curr['loc1'], dir)
                
                if is_illegal(child_loc, my_map) or is_ext_constrained(curr['parent']['loc1'], child_loc, curr['timestep'], constraintTable):
                    continue

                if is_conflicted(curr['loc0'], curr['parent']['loc0'], child_loc, curr['parent']['loc1']):
                    continue
                
                child = {'loc0': curr['loc0'], 
                        'loc1': child_loc,
                        'g_val': curr['g_val'] + 1, 
                        'h_val': curr['h_val'] + h_values['child_loc'],
                        'parent': curr['parent'],
                        'timestep': curr['timestep']}
                
                if (child['loc0'], child['loc1'], child['timestep']) in closed_list:
                    existing_node = closed_list[(child['loc0'], child['loc1'], child['timestep'])]
                    if compare_nodes(child, existing_node):
                        closed_list[(child['loc0'], child['loc1'], child['timestep'])] = child
                        push_node(open_list, child)
                else:
                    closed_list[(child['loc0'], child['loc1'], child['timestep'])] = child
                    push_node(open_list, child)

    return None