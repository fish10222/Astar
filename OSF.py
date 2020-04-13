def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]

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
def OSF (fval, curr_node, h_values, my_map, constraint_table, agent):
    """
        f_next =  fval + min(∆f)
        returns ([list of locations], f_next)
    """
    newlocs = []
    curr_time = curr_node['timestep']
    curr_hval = curr_node['h_val']
    current_loc = curr_node['loc']
    f_next = 9999999999999
    for dir in range(5):
        next_loc = move(current_loc, dir)
        if is_illegal(next_loc, my_map) or is_ext_constrained(current_loc, next_loc, curr_time + 1, constraint_table):
            continue
        next_hval = h_values[agent][next_loc]
        delta_h = next_hval - curr_hval
        delta_f = lookup_table[delta_h]
        f_next_node = fval + delta_f
        if f_next_node == fval:         #if next node matches the f_value, we put in list
            newlocs.append(next_loc)
        f_next = min(f_next, f_next_node)   #set f_next to samllest f
    if f_next >= 9999999999999:
        f_next = -1
    answers = (newlocs, f_next)
    return answers