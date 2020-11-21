import gym
import gym_warehouse
import math

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0


    def __eq__(self, other):
        return self.position == other.position









def astar(graph, start, end):
    dr = [0, 0, -1, 1]
    dc = [-1, 1, 0, 0]
    startNode = Node(None, start)
    endNode = Node(None, end)
    startNode.g = startNode.h = startNode.f = 0
    endNode.g = endNode.h = endNode.f = 0

    frontier = []
    explored = []
    frontier.append(startNode)

    while frontier:

        current_node = frontier[0]
        min = frontier[0].f

        for nodes in frontier:
            if (nodes.f < min):
                min = nodes.f
                current_node = nodes

        frontier.remove(current_node)
        explored.append(current_node)

        if current_node == endNode:
            return path(current_node)

        for i in range(4):
            cr = [current_node.position[0]+ dr[i], current_node.position[1] + dc[i]]
            if cr[0] < 0 or cr[1] < 0 or cr[0] >= len(graph) or cr[1] >= len(graph[0]):
                continue

            if graph[cr[0]][cr[1]] == '*':
                continue

            neighborNode = Node(current_node, cr)

            control1 = False
            for nodes in explored:
                if neighborNode == nodes:
                    control1 = True

            if(control1):
                continue


            neighborNode.g = current_node.g + 1
            neighborNode.h = math.sqrt(((neighborNode.position[0] - endNode.position[0]) ** 2) + ( (neighborNode.position[1] - endNode.position[1]) ** 2))
            neighborNode.f = neighborNode.g + neighborNode.h

            control = False
            for nodes in frontier:
                if( neighborNode == nodes):
                    if( neighborNode.g < nodes.g):
                        nodes.parent = current_node
                        nodes.g = neighborNode.g
                        nodes.f = nodes.g + nodes.h
                    control = True



            if(not control):
                frontier.append(neighborNode)






    return None




def path(node):
    path_back = []

    while node:
        path_back.append(node)
        node = node.parent
    return list(reversed(path_back))


def run_agent():
    # create the Gym environment
    env = gym.make('singlerobot-warehouse-v0')
    s = env.look()

    row = len(s)
    column = len(s[0])
    current = [-1, -1]
    end = [-1, -1]
    for i in range(row):
        for j in range(column):
            if(s[i][j] == 'a'):
                current[0] = i
                current[1] = j
            if (s[i][j] == 'A'):
                end[0] = i
                end[1] = j



    path = astar(s,current,end)

    for cost in path:
        print(cost.g)


    i = 1
    while True:
        env.render()    # you can used this for printing the environment

        # sense

        # think
        # ...

        # act

        if (path[i].position[0] > current[0]):
            ob, rew, done = env.step(env.ACTION_DOWN)
            current[0] = current[0] + 1

        elif (path[i].position[0] < current[0]):
            ob, rew, done = env.step(env.ACTION_UP)
            current[0] = current[0] - 1


        elif (path[i].position[1] < current[1]):
            ob, rew, done = env.step(env.ACTION_LEFT)
            current[1] = current[1] - 1


        elif (path[i].position[1] > current[1]):
            ob, rew, done = env.step(env.ACTION_RIGHT)
            current[1] = current[1] + 1



        i = i + 1

        if done:
            break

    env.close()


if __name__ == "__main__":
    run_agent()


