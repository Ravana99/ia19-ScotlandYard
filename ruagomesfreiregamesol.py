# Group 18 - Joao Fonseca 89476, Tomas Lopes 89552

import math
import pickle
import time
import heapq


class Node:

    def __init__(self, parent=None, transport=None, index=None, tickets=[math.inf, math.inf, math.inf], distance = 0):
        self.parent = parent
        self.transport = transport
        self.index = index
        self.tickets = tickets
        self.distance = distance

        self.f = 0
        self.g = 0
        self.h = 0

    def __eq__(self, other):
        return self.index == other.index

    def __lt__(self, other):
        if self.f < other.f:
            return True
        elif self.f == other.f and self.distance < other.distance:
            return True
        return False

    def __repr__(self):
        return str((self.index, self.f))


class SearchProblem:

    def __init__(self, goal, model, auxheur=[]):

        self.goal = goal
        self.model = model
        self.auxheur = auxheur

    def search(self, init, limitexp=2000, limitdepth=10, tickets=[math.inf, math.inf, math.inf], anyorder = False):
        nagents = len(init)
        heuristics = [None] * nagents
        for i in range(nagents):
            heuristics[i] = BFS(self.model, self.goal[i])
        solution = [None] * nagents
        found = astar(self.model, init, self.goal, heuristics, tickets, limitexp, limitdepth, self.auxheur, 0, solution) # MISSING EXP_COUNT
        solution = mergeSolutions(solution)
        if found:
            return solution
        else:
            return []

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

# Test limitexp and limitdepth
def astar(model, init, goal, heuristics, tickets, limitexp, limitdepth, auxheur, agent, solution): # MISSING EXP_COUNT AND DEPTH_COUNT

    nagents = len(init)
    if agent == nagents: # Stopping condition
        return True

    init_node = Node(index=init[agent], tickets = tickets)
    heuristic = heuristics[agent]
    init_node.h = heuristic[init_node.index]
    init_node.f = init_node.h
    goal_node = Node(index=goal[agent])

    model_length = len(model)
    isexp = [(False, math.inf)] * model_length

    exp_nodes = []
    heapq.heapify(exp_nodes)

    heapq.heappush(exp_nodes, init_node)
    isexp[init_node.index] = (True, init_node.g)

    while len(exp_nodes) > 0:

        curr_node = heapq.heappop(exp_nodes)

        isexp[curr_node.index] = (False, curr_node.g)

        if curr_node == goal_node:
            if agent >= 1 and curr_node.g < len(solution[agent-1])-1:
                pass
            elif agent >= 1 and curr_node.g > len(solution[agent-1])-1:
                solution[agent-1] = None
                return False
            else:
                path = []
                current = curr_node
                while current is not None:
                    if current.transport != None:
                        path.append([[current.transport], [current.index]])
                    else:
                        path.append([[], [current.index]])
                    current = current.parent
                
                path = path[::-1] # Reverses path
                if agent >=1 and hasConflicts(agent, path, solution):
                    path = []
                else:
                    solution[agent] = path
                    nextagent = astar(model, init, goal, heuristics, curr_node.tickets, limitexp, limitdepth, auxheur, agent+1, solution)
                    if nextagent:
                        return True


        #if curr_node.g >= limitdepth:
            #continue

        #exp_count += 1

        #if exp_count >= limitexp:
            #continue

        children = []

        for el in model[curr_node.index]:
            new_node = Node(parent=curr_node, transport=el[0], index=el[1], tickets = curr_node.tickets.copy())
            if new_node.tickets[el[0]] > 0:
                new_node.tickets[el[0]] -= 1
                children.append(new_node)

        for child in children:

            child.g = curr_node.g + 1
            child.h = heuristic[child.index]
            child.f = child.g + child.h
            child.distance = distanceToTarget(auxheur[child.index+1], auxheur[goal_node.index+1])

            if isexp[child.index][0] and child.g > isexp[child.index][1]:
                continue

            heapq.heappush(exp_nodes, child)
            isexp[child.index] = (False, child.g)

    return False

def distanceToTarget(source, target):
    return math.sqrt((source[0]+target[0])*(source[0]+target[0]) + (source[1]+target[1])*(source[1]+target[1]))

def hasConflicts(agent, path, solution):
    for i in range(agent):
        prev = solution[i]
        for j in range(len(prev)):
            if prev[j][1] == path[j][1]:
                return True
    return False

def mergeSolutions(lst):
    sol = lst[0].copy()
    for path in range(1, len(lst)):
        for turn in range(len(lst[path])):
            sol[turn][0] += lst[path][turn][0]
            sol[turn][1] += lst[path][turn][1]

    return sol