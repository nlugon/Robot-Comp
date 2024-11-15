
thresholdWalkable = 8

obstaclesWeight = 100
ghWeight = 1

# let's implement the a* algorithm

def calcTrajectory(field = Field(),objective = (0,0)):
    data = getTestingData()

    field = data["field"]

    # field.showObstacles()

    start = field.robot.position
    end = objective

    path = astar(field.obstacleGrid, start, end)

    #show the path on the map
    for i in range(len(path)): 
        field.obstacleGrid[path[i][0]][path[i][1]] = -5 

    field.showObstacles()


class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.o = 0
        # self._f = 0

    @property
    def f(self):
        return (self.g + self.h) * ghWeight + self.o * obstaclesWeight


    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)
    # print("open list: ", open_list)

    # Loop until you find the end
    while len(open_list) > 0:
        # print("open list: ", open_list)
        
        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
                # print("current node: ", current_node.position)

        # print("current node: ", current_node.position, "length of open list: ", len(open_list), "length of closed list: ", len(closed_list))

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            print("found the goal")
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # Adjacent squares , (-1, -1), (-1, 1), (1, -1), (1, 1)
            # print("new position: ", new_position)

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                # print("out of range")
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] > thresholdWalkable:
                # print("not walkable")
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)


        
        # Loop through children
        for child in children:
            # print("child: ", child.position, f"child in closed list: {child in closed_list}", f"child in open list: {child in open_list}")

            # Child is on the closed list
            for closed in closed_list:
                if closed == child:
                    # print("child is on the closed list", child.position)
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2) 
            child.o = maze[child.position[0]][child.position[1]]
            # child.f = child.g + child.h

            # Child is already in the open list
            if child in open_list:
                # print("child is already in the open list", child.position)
                continue
            # for open_node in open_list:
            #     # print(child.position, open_node.position, child.position == open_node.position)
            #     if child == open_node:# and child.g > open_node.g:
            #         continue

            # Add the child to the open list
            # print("adding child to open list: ", child.position)
            open_list.append(child)