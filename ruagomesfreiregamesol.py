# Group 18 - Joao Fonseca 89476, Tomas Lopes 89552

import math
import pickle
import time
import heapq


class Node:

    def __init__(self, parent=None, transport=None, index=None, tickets=[math.inf, math.inf, math.inf], distance = 0):
        self.parent = parent        # Parent node
        self.transport = transport  # Transport used to get to the node
        self.index = index          # Node number
        self.tickets = tickets      # Tickets available while on this node
        self.distance = distance    # Euclidean distance to goal node (if anyorder: nearest goal node)

        # Heuristic related function values
        self.f = 0          
        self.g = 0
        self.h = 0

    def __eq__(self, other):
        return self.index == other.index

    # In the algorithm, the nodes are stored in a heap that takes into consideration the f value of each node.
    # In case of a tie, the algorithm chooses the node that is closest (in euclidean distance) to the goal node.
    def __lt__(self, other):
        if self.f < other.f:
            return True
        elif self.f == other.f and self.distance < other.distance:
            return True
        return False


class SearchProblem:

    def __init__(self, goal, model, auxheur=[]):

        self.goal = goal
        self.model = model
        self.auxheur = auxheur

    def search(self, init, limitexp=2000, limitdepth=10, tickets=[math.inf, math.inf, math.inf], anyorder = False):
        nagents = len(init)    # Number of agents
        heuristics = [None] * nagents

        # Calculates h value for each vertex, for each agent
        for i in range(nagents):
            heuristics[i] = BFS(self.model, self.goal[i])

        # If any order is valid, each node will have the minimum h value of all the h values calculated,
        # regardless of the agent
        if anyorder:
            heuristics = [getMinHeuristic(heuristics)] * nagents

        # The "solution" list stores a list of n paths, each one storing one agent's path to the goal
        solution = [None] * nagents
        found = astar(self.model, init, self.goal, heuristics, tickets, limitexp, 0,
                      limitdepth, self.auxheur, anyorder, 0, solution)
        if found:
            solution = mergeSolutions(solution)    # Converts solution to required format
            return solution
        else:
            return []

# Auxiliary function for heuristic calculations. This function calculates the minimum distance from
# all possible vertices to a certain target vertex by doing a Breadth-First Search,
# disregarding ticket limits and path conflicts, creating an admissible heuristic.
def BFS(model, init_index):

    model_length = len(model)
    visited = [False] * model_length
    parents = [None] * model_length
    hlist = [math.inf] * model_length

    queue = []

    queue.append(init_index)
    visited[init_index] = True
    parents[init_index] = None
    hlist[init_index] = 0

    while queue:

        parent = queue.pop(0)

        for el in model[parent]:
            child = el[1]
            if visited[child] == False:
                queue.append(child)
                visited[child] = True
                parents[child] = parent
                hlist[child] = hlist[parent] + 1

    return hlist

# Search algorithm (A*) - uses the previously calculated heuristics to find optimal paths from
# the goal nodes to the target nodes. Works recursively:
# The program first tries to find a path for agent 0. If it succeeds, it tries to find a path
# for the next agent. If it ever fails to find a path with the same amount of turns taken and
# no path conflicts, it backtracks to the previous agent, which then tries to find an alternative path.
# Returns true if it's able to find a viable path for all agents, storing each individual path in the
# list "solution", or false if otherwise.
def astar(model, init, goal, heuristics, tickets, limitexp, exp_count, limitdepth, auxheur, anyorder, agent, solution):

    nagents = len(init)

    # Stopping condition
    if agent == nagents:    
        return True

    # Initializing

    heuristic = heuristics[agent]

    init_node = Node(index=init[agent], tickets = tickets)
    init_node.h = heuristic[init_node.index]
    init_node.f = init_node.h

    goal_nodes = []
    if anyorder:
        for i in range(nagents):
            goal_nodes.append(Node(index=goal[i]))
    else:
        goal_nodes.append(Node(index=goal[agent]))

    model_length = len(model)
    isopen = [(False, math.inf)] * model_length    # Auxiliary list used to check if a node is open

    # Heap used to store open nodes. Popping a node will yield the open node with the smallest h value,
    # and in case of a tie, it chooses the node with the smallest distance to target
    open_nodes = []
    heapq.heapify(open_nodes)    

    heapq.heappush(open_nodes, init_node)
    isopen[init_node.index] = (True, init_node.g)

    while len(open_nodes) > 0:    # Iterates open nodes

        # Checks if the node expansion limit was reached, stopping the search
        if exp_count >= limitexp:
            return False

        curr_node = heapq.heappop(open_nodes)

        isopen[curr_node.index] = (False, curr_node.g)

        # Reached goal
        if curr_node in goal_nodes:

            # Found path that is too short in comparison to previously calculated paths
            # Needs recalculation to obtain longer path
            if agent >= 1 and curr_node.g < len(solution[agent-1])-1:
                pass

            # Found path that is too long in comparison to previously calculated paths
            # Needs to backtrack to the previous agent and erase its path
            elif agent >= 1 and curr_node.g > len(solution[agent-1])-1:
                solution[agent-1] = None
                return False

            else:
                # Generates possible path by backtracking from the target to the source
                path = []
                current = curr_node
                while current is not None:
                    if current.transport != None:
                        path.append([[current.transport], [current.index]])
                    else:
                        path.append([[], [current.index]])
                    current = current.parent
                path = path[::-1]    # Reverses path

                # Checks if multiple agents are on the same node on the same turn
                if agent >=1 and hasConflicts(agent, path, solution):
                    path = []

                # Found a viable path
                else:
                    # Adds path to solution
                    solution[agent] = path

                    # Tries to find the next agent's path
                    nextagent = astar(model, init, goal, heuristics, curr_node.tickets, limitexp, exp_count,
                                      limitdepth, auxheur, anyorder, agent+1, solution)
                    
                    # Successfully found viable paths for all subsequent agents
                    if nextagent:
                        return True

        # Checks if the depth limit was reached, not letting the node be expanded if that is the case
        if curr_node.g >= limitdepth:
            continue

        exp_count += 1

        # Generates the children of a node that are reachable, accounting for the number of tickets
        children = []
        for el in model[curr_node.index]:
            new_node = Node(parent=curr_node, transport=el[0], index=el[1], tickets = curr_node.tickets.copy())
            if new_node.tickets[el[0]] > 0:
                new_node.tickets[el[0]] -= 1
                children.append(new_node)

        # Calculates heuristic related function values for every child and opens each node
        for child in children:
            child.g = curr_node.g + 1
            child.h = heuristic[child.index]
            child.f = child.g + child.h

            mindistance = distanceToTargetSquared(auxheur[child.index-1], auxheur[goal_nodes[0].index-1])
            # In case any order is valid, saves the distance to only the nearest goal node
            for i in range(1, len(goal_nodes)):
                mindistance = min(mindistance, distanceToTargetSquared(auxheur[child.index-1], 
                                                                       auxheur[goal_nodes[i].index-1]))
            child.distance = mindistance

            # Optimization - ignores alternate paths to the same node that take more turns
            if isopen[child.index][0] and child.g > isopen[child.index][1]:
                continue

            heapq.heappush(open_nodes, child)
            isopen[child.index] = (True, child.g)

    return False

# In case any order is valid, the h values for all nodes will be the same for all agents,
# and will be the minimum of all h values calculated for a specific node
def getMinHeuristic(heuristics):
    lst = []
    for i in range(len(heuristics[0])):
        aux = math.inf
        for j in range(len(heuristics)):
            aux = min(aux, heuristics[j][i])
        lst.append(aux)
    return lst        

# Calculates euclidean distance (squared) between two nodes
def distanceToTargetSquared(source, target):
    return (source[0]+target[0])*(source[0]+target[0]) + (source[1]+target[1])*(source[1]+target[1])

# Checks if multiple agents are on the same node on the same turn to determine whether a newly calculated path is valid
def hasConflicts(agent, path, solution):
    for i in range(agent):
        prev = solution[i]
        for j in range(len(prev)):
            if prev[j][1] == path[j][1]:
                return True
    return False

# Turns a list of n individual paths (for each agent) into a single list with all the paths in the expected format
def mergeSolutions(lst):
    sol = lst[0].copy()
    for path in range(1, len(lst)):
        for turn in range(len(lst[path])):
            sol[turn][0] += lst[path][turn][0]
            sol[turn][1] += lst[path][turn][1]

    return sol