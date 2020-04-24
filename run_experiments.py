#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from EPEAstar import EPEAStarSolver
from EPEAstar_algorithm import epea_star
from Astar_coupled_std import a_star_coupled
from Astar_coupled_ODID import a_star_coupled_ODID
#from Astar import AstarSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost

SOLVER = "CBS"

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--agents', type=int, default=None,
                        help='Number of agents in the instance')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))
    parser.add_argument('--B', type=int, default=1000000,
                        help='The B for MA-CBS, defaults to CBS')

    args = parser.parse_args()


    result_file = open("results.csv", "w", buffering=1)
    if args.solver == "MACBS":
        time_total_A = 0
        expanded_total_A = 0
        generated_total_A = 0
        time_total_IDOD = 0
        expanded_total_IDOD = 0
        generated_total_IDOD = 0
        time_total_EPEA = 0
        expanded_total_EPEA = 0
        generated_total_EPEA = 0
    else:
        time_total = 0
        expanded_total = 0
        generated_total = 0
    for file in sorted(glob.glob(args.instance)):

        
        my_map, starts, goals = import_mapf_instance(file)
        
        

        if (len(starts) == args.agents) or (args.agents is None):
            print("***Import an instance***")
            print_mapf_instance(my_map, starts, goals)
            if args.solver == "MACBS":
                print("***Run MA-CBS with each of A*, A*+OD+ID and EPEA* ***")
                cbs = CBSSolver(my_map, starts, goals, args.B)
                paths, CPU_time_A, expanded_A, generated_A = cbs.find_solution(a_star_coupled, args.disjoint) #A*
                _, CPU_time_IDOD, expanded_IDOD, generated_IDOD = cbs.find_solution(a_star_coupled_ODID, args.disjoint) #ID OD
                _, CPU_time_EPEA, expanded_EPEA, generated_EPEA = cbs.find_solution(epea_star, args.disjoint) #EPEA*
                time_total_A += CPU_time_A
                expanded_total_A += expanded_A
                generated_total_A += generated_A
                time_total_IDOD += CPU_time_IDOD
                expanded_total_IDOD += expanded_IDOD
                generated_total_IDOD += generated_IDOD
                time_total_EPEA += CPU_time_EPEA
                expanded_total_EPEA += expanded_EPEA
                generated_total_EPEA += generated_EPEA
            else:
                if args.solver == "Astar":
                    print("***Run MA-CBS with Astar***")
                    cbs = CBSSolver(my_map, starts, goals, args.B)            
                    paths, CPU_time, expanded, generated = cbs.find_solution(a_star_coupled, args.disjoint)
                elif args.solver == "EPEA":
                    print("***Run MA-CBS with EPEAstar***")
                    cbs = CBSSolver(my_map, starts, goals, args.B)            
                    paths, CPU_time, expanded, generated = cbs.find_solution(epea_star, args.disjoint)
                elif args.solver == "IDOD":
                    print("***Run MA-CBS with A*+ID+OD***")
                    cbs = CBSSolver(my_map, starts, goals, args.B)            
                    paths, CPU_time, expanded, generated = cbs.find_solution(a_star_coupled_ODID, args.disjoint)
                else:
                    raise RuntimeError("Unknown solver!")
                time_total += CPU_time
                expanded_total += expanded
                generated_total += generated
        
            cost = get_sum_of_cost(paths)
            result_file.write("{},{}\n".format(file, cost))

            
            
            if not args.batch:
                print("***Test paths on a simulation***")
                animation = Animation(my_map, starts, goals, paths)
                # animation.save("output.mp4", 1.0)
                animation.show()

    if args.solver == "MACBS":
        print("Total CPU time (s) for A*:    {:.2f}".format(time_total_A))
        print("Total Expanded nodes for A*:  {}".format(expanded_total_A))
        print("Total Generated nodes for A*: {}".format(generated_total_A))
        print("Total CPU time (s) for A*+ID+OD:    {:.2f}".format(time_total_IDOD))
        print("Total Expanded nodes for A*+ID+OD:  {}".format(expanded_total_IDOD))
        print("Total Generated nodes for A*+ID+OD: {}".format(generated_total_IDOD))
        print("Total CPU time (s) for EPEA*:    {:.2f}".format(time_total_EPEA))
        print("Total Expanded nodes for EPEA*:  {}".format(expanded_total_EPEA))
        print("Total Generated nodes for EPEA*: {}".format(generated_total_EPEA))
    else:
        print("Total CPU time (s):    {:.2f}".format(time_total))
        print("Total Expanded nodes:  {}".format(expanded_total))
        print("Total Generated nodes: {}".format(generated_total))

    result_file.close()
