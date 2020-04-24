from single_agent_planner import compute_heuristics, get_sum_of_cost, a_star, get_location
from Astar_OD import a_star_OD

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
                # Return first collision
                return collisions
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

def build_constraints(pathAgent, targetAgent, path, ext_constraints):
    constraints = list()

    # Adding constraints
    for i in pathAgent:
        for k in range(len(targetAgent)):
            for j in range(len(path[i])):
                if targetAgent[k] != i:
                    constraint = {'agent': k,'loc': [path[i][j]],'timestep': j, 'goal': j == len(path[i]) - 1}
                    constraints.append(constraint)

                    if j != 0:    
                        constraint = {'agent': k, 'loc': [path[i][j], path[i][j-1]], 'timestep': j, 'goal': False}
                        constraints.append(constraint)
    
    # Adding external constraint
    for x in ext_constraints:
        for i in x['agents']:
            if i in targetAgent:
                constraint = {'agent': targetAgent.index(i), 'loc': x['loc'], 'timestep': x['timestep'], 'goal': False}
                constraints.append(constraint)

    return constraints

def a_star_coupled(my_map, start_locs, goal_locs, h_values, ext_constraints):
    agentCount = len(start_locs)


    # Assign each agent to a group
    agentGroupList = [i for i in range(agentCount)]     
    groupList = [[i] for i in range(agentCount)]
    past_conflicts = list() 
    paths = list()
    replanned = list()

    # Plan path for each group
    for i in range(agentCount):  # Find initial path for each agent
        path = a_star_OD(my_map, [start_locs[i]], [goal_locs[i]], [h_values[i]], [], [])[0]
        
        if path is None:
            raise BaseException('No solutions')            
            
        paths.append(path)

    collisions = detect_collisions(paths)
    print("ID starts")
    
    while collisions:
        a1 = collisions[0]['a1']
        a2 = collisions[0]['a2']
        g1 = groupList[agentGroupList[a1]]
        g2 = groupList[agentGroupList[a2]]
        g1_solution_found = False
        g2_solution_found = False
        conflict_group = None
        
        print()
        print("Resolving conflict between " + str(g1) + " and " + str(g2))
        # if not conflicted before
        if tuple(g1 + g2) not in past_conflicts:
            if tuple(g1) not in replanned:
                print("Replanning for " + str(g1))
                constraints = build_constraints(g2, g1, paths, ext_constraints)
                conflict_avoidance_table = build_constraints([i not in g1 for i in range(agentCount)], g1, paths, [])

                conflict_group = g1
                conflict_group_old_cost = sum([len(paths[i]) for i in conflict_group])

                new_path = a_star_OD(my_map, [start_locs[i] for i in g1], [goal_locs[i] for i in g1],
                    [h_values[i] for i in g1], constraints, conflict_avoidance_table)
                    
                if new_path:
                    if sum(len(path) for path in new_path) <= conflict_group_old_cost:
                        print("Replan successful for " + str(conflict_group))
                        
                        g1_solution_found = True
                        if len(replanned) >= 2:
                            replanned.pop(0)
                        replanned.append(tuple(g1))


            if not g1_solution_found:
                print("Replanning for " + str(g2))
                conflict_group = g2
                conflict_group_old_cost = sum([len(paths[i]) for i in conflict_group])
                
                conflict_avoidance_table = build_constraints([i not in g2 for i in range(agentCount)], g2, paths, [])
                constraints = build_constraints(g1, g2, paths, ext_constraints)
                
                new_path = a_star_OD(my_map, [start_locs[i] for i in g2], [goal_locs[i] for i in g2],
                    [h_values[i] for i in g2], constraints, conflict_avoidance_table)
                    
                if new_path:
                    if sum(len(path) for path in new_path) <= conflict_group_old_cost:
                        print("Replan successful for " + str(conflict_group))

                        g2_solution_found = True
                        if len(replanned) >= 2:
                            replanned.pop(0)
                        replanned.append(tuple(g2))
                
        
        if not g1_solution_found and not g2_solution_found:
            print("Replan unsuccessful, merging groups " + str(g1) + " and " +str(g2))
            conflict_group = merge_group(groupList, agentGroupList[a1], agentGroupList[a2], agentGroupList)
            conflict_avoidance_table = build_constraints([i for i in range(agentCount)], conflict_group, paths, [])
            constraints = build_constraints([], g2, [], ext_constraints)
            new_path = a_star_OD(my_map, [start_locs[i] for i in conflict_group], [goal_locs[i] for i in conflict_group],
                [h_values[i] for i in conflict_group], constraints, [])
            
            if len(replanned) >= 2:
                replanned.pop(0)

            replanned.append(tuple(g1 + g2))
        
        else:
            past_conflicts.append(tuple(g1 + g2))


        for i in range(len(conflict_group)):
            paths[conflict_group[i]] = new_path[i]
        
        collisions = detect_collisions(paths)

        # for collision in collisions:
        #     print(collision)

    print("Solution:")
    for path in paths:
        print(path)
    return paths