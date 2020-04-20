import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost, a_star
from Astar_coupled_std2 import a_star_coupled
############################################# TODO: check conflicts before extracting external conflict, so to make sure no internal constraints passed to low level
def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    # TODO: detect collision even when an agent has reached its goal.
    length = max(len(path1), len(path2))
    t = 0
    while t<length:
        # detect vertex collision
       
        if get_location(path1, t) == get_location(path2, t):
            col = {'loc': [get_location(path1, t)], 'timestep': t}
            
            return col
        # detect edge collision
        if t > 0:
            if get_location(path1, t) == get_location(path2, t-1) and get_location(path1, t-1) == get_location(path2, t):
                col = {'loc': [get_location(path1,t-1), get_location(path1, t)], 'timestep': t}
                return col
        t = t + 1


def detect_collisions(paths, meta_agents): 
    # a meta conflict is a tuple (x_bar,y_bar,v,t) where an individual agent x′∈ x_bar and an individual agent y′∈ y_bar 
    # both occupy vertex v at time point t.
    cols = []
    for i in range(len(meta_agents)):
        for j in range(i+1, len(meta_agents)):
            for ai in meta_agents[i]:                
                for aj in meta_agents[j]:
                    col = detect_collision(paths[ai], paths[aj])
                    if col is not None:
                        cols.append({'a1': meta_agents[i], 'a2': meta_agents[j], 'loc': col['loc'], 'timestep': col['timestep']})
                        
                        
    return cols


def standard_splitting(collision): # follows 8.4.1 paragraph 4
    ##############################
    # A meta constraint for a meta-agent x_bar is a tuple (x_bar,ˆx,v,t) where a subset of agents ˆx ⊆ x_bar are prohibited from
    # occupying vertex v at timestep t
    constraints = []
    if len(collision['loc']) == 1:
        #vertex collision
        constraints.append({'meta_agent': collision['a1'], 'agents': None, 'loc': collision['loc'], 'timestep': collision['timestep'], 'collided_with': collision['a2']})
        constraints.append({'meta_agent': collision['a2'], 'agents': None, 'loc': collision['loc'], 'timestep': collision['timestep'], 'collided_with': collision['a1']})
    else:
        # edge collision
        loc = collision['loc'][:]
        loc.reverse()
        constraints.append({'meta_agent': collision['a1'], 'agents': None, 'loc': collision['loc'], 'timestep': collision['timestep'], 'collided_with': collision['a2']})
        constraints.append({'meta_agent': collision['a2'], 'agents': None, 'loc': loc, 'timestep': collision['timestep'], 'collided_with': collision['a1']})
    return constraints


def extract_external_constraints(external_constraints, agents): # takes external constraints meta_agent and outputs same constraints but in format that coupled a* needs  
    # !!!!!important!!!!!
    # TODO:
    # remember to exclude constraints formed because of the conflicts(collisions) between any pair of agents/meta-agents inside the meta-agent we are looking at 
    
    
    constraints = []
    agent_index = dict()
    for i in range(len(agents)):
        agent_index[agents[i]] = i
    for xc in external_constraints:
        new_constraint = {'loc': xc['loc'], 'timestep': xc['timestep']}
        if xc['agents'] is None:
            new_constraint['agents'] = list(range(len(agents)))
        else:
            new_constraint['agents'] = [agent_index[a] for a in xc['agents']]
        constraints.append(new_constraint)
    return constraints

def metaconstr2constr(meta_constraint): # output a list of constraints taken from a meta-constraint which single a* takes as input 
    if meta_constraint['agents'] is None:
        return [{'agent': meta_constraint['meta_agent'][i], 'loc': meta_constraint['loc'], 'timestep': meta_constraint['timestep']} for i in range(len(meta_constraint['meta_agent']))]
    else:
        return [{'agent': meta_constraint['agents'][i], 'loc': meta_constraint['loc'], 'timestep': meta_constraint['timestep']} for i in range(len(meta_constraint['agents']))]



class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.B = 2 #MA-CBS
        
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        
        self.CM = [[0 for i in range(self.num_of_agents)] for i in range(self.num_of_agents)] #MA-CBS

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['meta_collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def shouldMerge(self, ai, aj):
        acc = 0
        if type(ai) == int:
            ai = [ai]
        if type(aj) == int:
            aj = [aj]  
        for x in ai:
            for y in aj:
                if x < y:
                    acc = acc + self.CM[x][y]
                else:
                    acc = acc + self.CM[y][x]
        if acc > self.B:
            return True 
        return False

    def updateCM(self, collision):
        a1 = collision['a1']
        a2 = collision['a2']
        for x in a1:
            for y in a2:
                if x > y:
                    self.CM[y][x] = self.CM[y][x] + 1
                else:
                    self.CM[x][y] = self.CM[x][y] + 1
        pass

    def merge_external_constraints(self, meta_agent1, meta_agent2, constraints): 
        new_constraints = []
        for constraint in constraints:
            if constraint['meta_agent'] == meta_agent1:
                if constraint['agents'] is None: # 8.4.1 paragraph 2
                    new_constraint = {'meta_agent': meta_agent1+meta_agent2, 'agents': meta_agent1, 'loc': constraint['loc'], 'timestep': constraint['timestep'], 'collided_with': constraint['collided_with']}
                    
                else:
                    new_constraint = {'meta_agent': meta_agent1+meta_agent2, 'agents': constraint['agents'], 'loc': constraint['loc'], 'timestep': constraint['timestep'],'collided_with': constraint['collided_with']}
                    
            elif constraint['meta_agent'] == meta_agent2:
                if constraint['agents'] is None: # 8.4.1 paragraph 2
                    new_constraint = {'meta_agent': meta_agent1+meta_agent2, 'agents': meta_agent2, 'loc': constraint['loc'], 'timestep': constraint['timestep'],'collided_with': constraint['collided_with']}
        
                else:
                    new_constraint = {'meta_agent': meta_agent1+meta_agent2, 'agents': constraint['agents'], 'loc': constraint['loc'], 'timestep': constraint['timestep'],'collided_with': constraint['collided_with']}
            else: 
                continue
            if not set(new_constraint['collided_with']).issubset(set(new_constraint['meta_agent'])): # don't output internal constraints
                new_constraints.append(new_constraint)
        return new_constraints

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths [{'agents' = [a1, a2], 'loc' = [<location(s)>]}, 'timestep': #]
        root = {'cost': 0,
                'meta_agents': [[i] for i in range(self.num_of_agents)], #MA-CBS
                'meta_constraints': [], # MA-CBS
                'constraints': [], # for single agent astar
                'paths': [],
                'meta_collisions': []} #MA-CBS
        for i in range(self.num_of_agents):  # Find initial path for each agent
            #path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          #i, root['constraints'])
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], 0, [])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['meta_collisions'] = detect_collisions(root['paths'], root['meta_agents'])
        self.push_node(root)

        # Task 3.1: Testing
        #print(root['meta_collisions'])

        # Task 3.2: Testing
        #for collision in root['meta_collisions']:
            #print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        count = 0
        closed_list = []
        while len(self.open_list) > 0:
            P = self.pop_node()            # P <- best node from OPEN // lowest solution cost
            #print(P)
            
            closed_list.append({'number': count, 'node': P})
            if len(P['meta_collisions']) == 0:  # if P has no conflict then
                self.print_results(P)
                #print(self.CM)
                return P['paths']          #    return P.solution // P is goal
            
            meta_collision = P['meta_collisions'][0]  # C <- first conflict(ai, aj, v, t) in P
            self.updateCM(meta_collision)
        ######## MA-CBS only(tons of work left)
            if self.shouldMerge(meta_collision['a1'], meta_collision['a2']):
                #print(P)
                #merge(meta_collision)
                a_meta = meta_collision['a1'] + meta_collision['a2'] #merge(collision['a1'], collision['a2'])
                # update meta agents
                
                P['meta_agents'] = P['meta_agents'][:]
                P['meta_agents'].remove(meta_collision['a1'])   
                P['meta_agents'].remove(meta_collision['a2'])
                P['meta_agents'].append(a_meta)

                common_ext_constr = self.merge_external_constraints(meta_collision['a1'], meta_collision['a2'], P['meta_constraints'])
                # update P.constraints (replace old constraints of the conflicted agents with new constraints)



                P['meta_constraints'] = P['meta_constraints'][:]
                for c in P['meta_constraints']:
                    if c['meta_agent'] == meta_collision['a1'] or c['meta_agent'] == meta_collision['a2']:
                        
                        P['meta_constraints'].remove(c)
                
                P['meta_constraints'] = P['meta_constraints'] + common_ext_constr
                
                # update P.solution (invoke low level(a_meta))
                meta_starts = []
                meta_goals = []
                meta_heuristics = []
                meta_constraints = extract_external_constraints(common_ext_constr, a_meta) # these are the constraints of the new meta-agent with outside agents
                for i in a_meta:
                    meta_goals.append(self.goals[i])
                    meta_starts.append(self.starts[i])
                    meta_heuristics.append(self.heuristics[i])
                
                meta_sol = a_star_coupled(self.my_map, meta_starts, meta_goals, meta_heuristics, meta_constraints)
                if meta_sol is not None:
                    for i in a_meta:
                        P['paths'][i] = meta_sol.pop(0)
                    P['cost'] = get_sum_of_cost(P['paths'])
                    P['meta_collisions'] = detect_collisions(P['paths'], P['meta_agents'])
                    #print(P)
                    self.push_node(P)
                continue
        ########
            #print(P)
            meta_constraints = standard_splitting(meta_collision)  ## 
            
            for meta_constraint in meta_constraints:
                Q = {'cost': 0,
                     'meta_agents': P['meta_agents'],
                     'meta_constraints': P['meta_constraints'] + [meta_constraint],
                     #'constraints': P['constraints'] + [metaconstr2constr(meta_constraint)],
                     'constraints': [],
                     'paths': P['paths'][:],
                     'meta_collisions': []}
                for c in Q['meta_constraints']:
                    Q['constraints'] = Q['constraints'] + metaconstr2constr(c)
                meta_agent = meta_constraint['meta_agent']
                ##### find individual paths for agents in ai, if any fails to get a solution, then abort node Q
                '''
                for a in ai:
                    path = a_star(self.my_map, self.starts[a], self.goals[a], self.heuristics[a],
                                a, Q['constraints'])
                    if path is not None:
                        Q['paths'][a] = path[:]
                        Q['meta_collisions'] = detect_collisions(Q['paths'], Q['meta_agents'])
                        Q['cost'] = get_sum_of_cost(Q['paths'])
                    else:
                        break
                            '''

                ##### find optimal paths for the meta-agent, if no solution then abort node Q
                meta_starts = [] #start locs, ordered as in meta_agent
                meta_goals = [] #goal locs, ordered as in meta_agent
                meta_heuristics = [] #h values, ordered as in meta_agent
                meta_ext_constraints = []# a_star_coupled uses external constraints only, not indexed
                for c in Q['meta_constraints']:
                    if c['meta_agent'] == meta_agent:
                        meta_ext_constraints.append(c)
                for i in meta_agent:
                    meta_goals.append(self.goals[i])
                    meta_starts.append(self.starts[i])
                    meta_heuristics.append(self.heuristics[i])
                if len(meta_agent) > 1:
                    paths = a_star_coupled(self.my_map, meta_starts, meta_goals, meta_heuristics, extract_external_constraints(meta_ext_constraints, meta_agent))
                else:
                    paths = [a_star(self.my_map, meta_starts[0], meta_goals[0], meta_heuristics[0], 0, extract_external_constraints(meta_ext_constraints, meta_agent))]
                if paths is not None:
                    for a in meta_agent:                      
                        Q['paths'][a] = paths.pop(0)[:]

                    Q['cost'] = get_sum_of_cost(Q['paths'])
                    Q['meta_collisions'] = detect_collisions(Q['paths'], Q['meta_agents'])
                    self.push_node(Q)
        
        self.print_results(root)
        return root['paths']


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
