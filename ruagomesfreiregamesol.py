import math
import pickle
import time
import heapq


class Node:

    def __init__(self, parent=None, transport=None, index=None):
        self.parent = parent
        self.transport = transport
        self.index = index

        self.f = 0
        self.g = 0
        self.h = 0

    def __eq__(self, other):
        return self.index == other.index

    def __lt__(self, other):
        return self.f < other.f

    def __repr__(self):
        return str(self.index)


class SearchProblem:

  def __init__(self, goal, model, auxheur=[]):
    self.goal = goal
    self.model = model
    self.auxheur = auxheur
    pass

  def search(self, init, limitexp=2000, limitdepth=10, tickets=[math.inf, math.inf, math.inf]):

    lst = astar(self.model, init, self.goal)
    print(lst)
    return lst


def astar(model, init, goal):

    init_node = Node(index=init[0])
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

        # Test (just before expansion)
        if curr_node == goal_node:
            path = []
            current = curr_node
            while current is not None:
                if current.transport != None:
                    path.append([[current.transport],[current.index]])
                else:
                    path.append([[], [current.index]])
                current = current.parent
            return path[::-1]  # Return reversed path

        children = []

        for el in model[curr_node.index]:
            new_node = Node(parent=curr_node, transport=el[0], index=el[1])
            children.append(new_node)

        for child in children:

            if isclosed[child.index]:
                continue

            child.g = curr_node.g + 1
            child.h = 0 # Heuristic missing
            child.f = child.g + child.h

            if isopen[child.index][0] and child.g > isopen[child.index][1]:
                continue

            heapq.heappush(open_nodes, child)
            isopen[child.index] = (False, child.g)


# // A* (star) Pathfinding
# // Initialize both open and closed list
# let the openList equal empty list of nodes
# let the closedList equal empty list of nodes
# // Add the start node
# put the startNode on the openList (leave it's f at zero)
# // Loop until you find the end
# while the openList is not empty
    # // Get the current node
    # let the currentNode equal the node with the least f value
    # remove the currentNode from the openList
    # add the currentNode to the closedList
    # // Found the goal
    # if currentNode is the goal
        # Congratz! You've found the end! Backtrack to get path
    # // Generate children
    # let the children of the currentNode equal the adjacent nodes

    # for each child in the children
        # // Child is on the closedList
        # if child is in the closedList
            # continue to beginning of for loop
        # // Create the f, g, and h values
        # child.g = currentNode.g + distance between child and current
        # child.h = distance from child to end
        # child.f = child.g + child.h
        # // Child is already in openList
        # if child.position is in the openList's nodes positions
            # if the child.g is higher than the openList node's g
                # continue to beginning of for loop
        # // Add the child to the openList
        # add the child to the openList

""" class Node():
    # A node class for A* Pathfinding

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    # Returns a list of tuples as a path from the given start to the given end in the given maze

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    goal_node = Node(None, end)
    goal_node.g = goal_node.h = goal_node.f = 0

    # Initialize both open and closed list
    open_nodes = []
    closed_nodes = []

    # Add the start node
    open_nodes.append(start_node)

    # Loop until you find the end
    while len(open_nodes) > 0:

        # Get the current node
        curr_node = open_nodes[0]
        curr_index = 0
        for index, item in enumerate(open_nodes):
            if item.f < curr_node.f:
                curr_node = item
                curr_index = index

        # Pop current off open list, add to closed list
        open_nodes.pop(curr_index)
        closed_nodes.append(curr_node)

        # Found the goal
        if curr_node == goal_node:
            path = []
            current = curr_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]  # Return reversed path

        # Generate children
        children = []
        # Adjacent squares
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:

            # Get node position
            node_position = (
                curr_node.position[0] + new_position[0], curr_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) - 1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(curr_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_nodes:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = curr_node.g + 1
            child.h = ((child.position[0] - goal_node.position[0]) **
                       2) + ((child.position[1] - goal_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_nodes:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_nodes.append(child) """
