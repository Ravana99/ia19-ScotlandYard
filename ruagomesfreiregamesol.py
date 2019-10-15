# Group 18 - Joao Fonseca 89476, Tomas Lopes 89552

import math
import pickle
import time
import heapq


class Node:

    def __init__(self, parent=None, transport=None, index=None, tickets=[math.inf, math.inf, math.inf]):
        self.parent = parent
        self.transport = transport
        self.index = index

        self.tickets = tickets

        self.f = 0
        self.g = 0
        self.h = 0

    def __eq__(self, other):
        return self.index == other.index

    def __lt__(self, other):
        return self.f < other.f

    def __repr__(self):
        return str((self.index, self.f))


class SearchProblem:

    def __init__(self, goal, model, auxheur=[]):

        self.goal = goal
        self.model = model
        self.auxheur = auxheur

    def search(self, init, limitexp=2000, limitdepth=10, tickets=[math.inf, math.inf, math.inf]):

        inverted_model = invertModel(self.model)
        heuristic = BFS(inverted_model, self.goal[0], tickets)
        lst = astar(self.model, init, self.goal, heuristic, tickets, limitexp, limitdepth)
        return lst

    def getHeur(self, tickets):

        inverted_model = invertModel(self.model)
        return BFS(inverted_model, self.goal[0], tickets)


def invertModel(model):

    inverted = []
    for i in range(len(model)):
        inverted.append([])

    for i, el in enumerate(model):
        for el2 in el:
            inverted[el2[1]].append([el2[0], i])

    return inverted


def BFS(model, init_index, tickets):

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
def astar(model, init, goal, heuristic, tickets, limitexp, limitdepth):

    exp_count = 0

    init_node = Node(index=init[0], tickets = tickets)
    goal_node = Node(index=goal[0])

    model_length = len(model)
    isopen = [(False, math.inf)] * model_length
    isclosed = [False] * model_length

    open_nodes = []
    heapq.heapify(open_nodes)

    heapq.heappush(open_nodes, init_node)
    isopen[init_node.index] = (True, init_node.g)

    while len(open_nodes) > 0:

        curr_node = heapq.heappop(open_nodes)

        isopen[curr_node.index] = (False, curr_node.g)
        isclosed[curr_node.index] = True

        if curr_node == goal_node:
            path = []
            current = curr_node
            while current is not None:
                if current.transport != None:
                    path.append([[current.transport], [current.index]])
                else:
                    path.append([[], [current.index]])
                current = current.parent
            return path[::-1]  # Return reversed path

        if curr_node.g >= limitdepth:
            continue

        exp_count += 1

        if exp_count >= limitexp:
            continue

        children = []

        for el in model[curr_node.index]:
            new_node = Node(parent=curr_node, transport=el[0], index=el[1], tickets = curr_node.tickets.copy())
            if new_node.tickets[el[0]] > 0:
                new_node.tickets[el[0]] -= 1
                children.append(new_node)

        for child in children:

            if isclosed[child.index]:
                continue

            child.g = curr_node.g + 1
            child.h = heuristic[child.index]
            child.f = child.g + child.h

            if isopen[child.index][0] and child.g > isopen[child.index][1]:
                continue

            heapq.heappush(open_nodes, child)
            isopen[child.index] = (False, child.g)