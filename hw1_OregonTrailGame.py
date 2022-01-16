import math
# define node's initial information
class Node():
    def __init__(self, state, parent, M_value):
        self.state = state
        self.parent = parent
        self.diagonal = False  # initial: this node isn't diagonal to its parent node
        self.Mvalue = M_value
        self.mud = 0
        self.rock = 0
        self.cost = 0  # path cost (g(n))
        self.distance = 0  # admissible heuristic (h(n))
        self.estimate = 0  # f(n) only used for sorting
    
    def update_rm(self):
        if self.Mvalue < 0:
            self.rock = abs(self.Mvalue)
        else:
            self.mud = self.Mvalue

    def update_distance(self, target):  # update h(n)
        self.distance = 9 * math.sqrt(abs(self.state[0] - target[0]) ** 2 + abs(self.state[1] - target[1]) ** 2)

    def update_estimate(self):  # update f(n)
        self.estimate = self.cost + self.distance

    def path_cost(self, algorithm):
        """
        Updated the path cost of the already recorded(may not added) child node.
        """
        if self.parent is not None:
            if algorithm == "BFS":
                cost = 1
            elif algorithm == "UCS" or "A*":
                if self.diagonal:  # the child is diagonal to its parent
                    cost = 14
                else:
                    cost = 10
                if algorithm == "A*":
                    # decide the muddiness level
                    cost += self.mud
                    # decide the absolute height
                    cost += abs(self.rock - self.parent.rock)
            self.cost = cost + self.parent.cost
            self.update_estimate()
    

# define queue's operation
class QueueOper():
    def __init__(self):
        self.storage = []

    def add(self, node):
        self.storage.append(node)

    def delete(self, node):
        self.storage.remove(node)

    def empty(self):
    # TODO: simplify the function
        if len(self.storage) == 0:
            return True
        else:
            return False
    
    def remove(self):  # FIFO
        node = self.storage[0]
        self.storage = self.storage[1:]
        return node

    def contains_state(self, state):  # state: func. parameter
        for node in self.storage:
            if node.state == state:
                return node
        return None

    def sort(self, algorithm):
        if algorithm == "BFS" or "UCS":
            self.storage.sort(key = lambda x: x.cost)
        if algorithm == "A*":
            self.storage.sort(key = lambda x: x.estimate)


DIRECTORY = "input.txt"
information = {}

def load_map():
    """
    Load data from input text files into memory.
    """
    with open(DIRECTORY, "r") as f:
        lines = f.readlines()

    # chosen search algorithm
    information["algorithm"] = lines[0].rstrip()
    
    # node map
    information["row"] = int(lines[1].split()[1])
    information["col"] = int(lines[1].split()[0])
    rawmap = [i.split() for i in lines[-information["row"]:]]
    nodemap = []
    for i in range(len(rawmap)): 
        nodemap.append(list(map(int, rawmap[i])))
    information["map"] = nodemap

    # starting point
    information["start"] = (int(lines[2].split()[1]), int(lines[2].split()[0]))

    # highest rock that can climb
    information["rock"] = int(lines[3])

    # end point list, e.g: [(1, 1), (6, 3)]
    target_num = int(lines[4])
    target = []
    for i in range(target_num):
        target.append((int(lines[i + 5].split()[1]), int(lines[i + 5].split()[0])))
    information["end"] = target


def shortest_path(begin, end):
    """
    Returns the node list of the shortest path between begin and end.

    If no possible path, returns FAIL.
    """
    # do not calculate the 1st cell's cost
    start = Node(state = begin, parent = None, M_value = information["map"][begin[0]][begin[1]])  # make node
    start.update_rm()
    # TODO: start.update_distance(end)?

    queue = QueueOper()
    queue.add(start)

    explored = QueueOper()

    while True:
        if queue.empty():  # no accessible step
            return None
        currnode = queue.remove()  # not empty, call remove function

        # goal test
        if currnode.state == end:  # as for this strategy, the goal will always at the last position(?)
            return currnode

        children = neighbor_cells(currnode.state)  # [((1, 2), 40, 10), ...]
        while len(children) != 0:
            cell = children.pop(0)  # remove the first cell, and return it
            child = Node(state = cell[0], parent = currnode, M_value = cell[1])  # create a node
            if cell[2] == 14:
                child.diagonal = True
            child.update_rm()
            child.update_distance(end)
            # whether add or not, always record the node and cost
            child.path_cost(information["algorithm"])

            if queue.contains_state(cell[0]) == None and explored.contains_state(cell[0]) == None:
                # if is unaccessible, not add the node; else is accessible or mud, always add
                if abs(child.rock - child.parent.rock) > information["rock"]:
                    continue
                queue.add(child)  # add node to queue
            # queue already has this cell, which means there exists a loop(another path go through this node)
            elif queue.contains_state(cell[0]) != None:  # use state to decide whether exists or not
                # currnode is the current child's parent
                node = queue.contains_state(cell[0])  # already added
                if child.cost < node.cost:
                    queue.add(child)
            elif explored.contains_state(cell[0]) != None:  # avoid loop
                node = explored.contains_state(cell[0])
                if child.cost < node.cost:
                    explored.delete(node)  # delete the node in explored
                    queue.add(child)  # add it to the queue(will explore)
        explored.add(currnode)
        queue.sort(information["algorithm"])


def neighbor_cells(currnode):
    """
    Returns a list of coordinates and M pairs that connect to the current node

    sorting by increasing order of M value.
    """
    neighbor = []  # [((row, colomn), M value, diagonal), ...]
    for i in range(currnode[0] - 1, currnode[0] + 2):
        for j in range(currnode[1] - 1, currnode[1] + 2):
            if (i, j) == currnode:
                continue
            if 0 <= i < information["row"] and 0 <= j < information["col"]:
                if i != currnode[0] and j != currnode[1]:
                    neighbor.append(((i, j), information["map"][i][j], 14))
                else:
                    neighbor.append(((i, j), information["map"][i][j], 10))
    neighbor.sort(key = lambda x: x[1])
    return neighbor


def find_path(node):
    """
    Return solution path(string format) by tracing parent of the end node.
    """
    solution = []
    while True:
        solution.append(node.state)
        if node.parent == None:
            break
        node = node.parent
    solution.reverse()

    # convert solution list to formatted string
    sol_str = ""
    for coordinates in solution:
        sol_str += f"{coordinates[1]},{coordinates[0]} "
    sol_str = sol_str.strip()
    return sol_str


def main():
    load_map()

    solutions = []
    for target in information["end"]:
        node = shortest_path(information["start"], target)
        if node == None:
            solutions.append("FAIL")
        else:
            solutions.append(find_path(node))
    
    solutions1 = enumerate(solutions, start=1)
    with open("output.txt", "w") as f:
        for i, solution in solutions1:
            for coordinates in solution:
                f.write(coordinates)
            # avoid the new end line
            # if not solutions[-1] in solution:
            if i < len(solutions):
                f.write("\n")


if __name__ == "__main__":
    main()

