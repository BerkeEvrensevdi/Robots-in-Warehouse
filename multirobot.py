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
        self.time = 0


    def __eq__(self, other):
        return self.position == other.position




def astar(graph, start,  end, allpaths):
    dr = [0, 0, -1, 1]
    dc = [-1, 1, 0, 0]
    startNode = Node(None, start)
    endNode = Node(None, end)
    startNode.g = startNode.h = startNode.f = startNode.time = 0
    endNode.g = endNode.h = endNode.f = endNode.time = 0

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

        frontier.remove(current_node) # frontier gibi calisiyor
        explored.append(current_node) # explored gibi calisiyor

        if current_node == endNode:
            return path(current_node)


        count = 0
        control4 = False
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
            neighborNode.time = current_node.time + 1
            neighborNode.h = math.sqrt(((neighborNode.position[0] - endNode.position[0]) ** 2) + ( (neighborNode.position[1] - endNode.position[1]) ** 2))
            neighborNode.f = neighborNode.g + neighborNode.h


            control = False
            for nodes in frontier:
                if( neighborNode == nodes and neighborNode.time == nodes.time):
                    if( neighborNode.g < nodes.g):
                        nodes.parent = current_node
                        nodes.g = neighborNode.g
                        nodes.f = nodes.g + nodes.h
                        nodes.time = neighborNode.time
                    control = True

            control2 = False
            control3 = False
            if allpaths:
                for pth in allpaths:
                    for node in pth:

                        if(node.time == neighborNode.time and node.position == neighborNode.position):
                            control3 = True
                            control4 = True

                        elif(node.time == neighborNode.time  and node.parent == neighborNode and node == neighborNode.parent):
                            control2 = True






            if(not control and not control2 and not control3):
                frontier.append(neighborNode)
                count = count + 1


        if (count == 0 and control4):
            waitNode = Node(current_node, current_node.position)
            waitNode.time = current_node.time + 1
            waitNode.g = current_node.g
            waitNode.h = current_node.h
            waitNode.f = current_node.f
            frontier.insert(0, waitNode)





    return None




def path(node):
    path_back = []

    while node:
        path_back.append(node)
        node = node.parent
    return list(reversed(path_back))



def run_agent():
    # create the Gym environment
    env = gym.make('multirobot-warehouse-v0')
    s = env.look()
    row = len(s)
    column = len(s[0])
    currentAndEnd = {
       "a": [[None , None ],[ None , None ]],
       "b": [[None , None ],[ None , None ]],
       "c": [[None , None ],[ None , None ]],
       "d": [[None , None ],[ None , None ]]
    }


    for i in range(row):
        for j in range(column):
            if (s[i][j] == 'a'):
                currentAndEnd["a"][0] = [i ,j]

            elif (s[i][j] == 'A'):
                currentAndEnd["a"][1] = [i, j]

            elif (s[i][j] == 'b'):
                currentAndEnd["b"][0] = [i, j]

            elif (s[i][j] == 'B'):
                currentAndEnd["b"][1] = [i, j]

            elif (s[i][j] == 'c'):
                currentAndEnd["c"][0] = [i, j]

            elif (s[i][j] == 'C'):
                currentAndEnd["c"][1] = [i, j]

            elif (s[i][j] == 'd'):
                currentAndEnd["d"][0] = [i, j]

            elif (s[i][j] == 'D'):
                currentAndEnd["d"][1] = [i, j]






    #action_length = 0

    startAndEnd = []

    if  (currentAndEnd['a'][0][0]):
        startAndEnd.append(currentAndEnd['a'])
    if (currentAndEnd['b'][0][0]):
        startAndEnd.append(currentAndEnd['b'])
    if (currentAndEnd['c'][0][0]):
        startAndEnd.append(currentAndEnd['c'])
    if (currentAndEnd['d'][0][0]):
        startAndEnd.append(currentAndEnd['d'])

    """
    for x in currentAndEnd.values():
        if x:
            startAndEnd.append(x)
    #print(startAndEnd)
    """
    current = []

    for x in startAndEnd:
        current.append(x[0])

    #print(current)

    allpaths = []

    for r in startAndEnd:
        pathr = astar(s, r[0], r[1], allpaths)
        if pathr:
            allpaths.append(pathr)
        else:
            return False

    """
    for nm in allpaths:
        for node in nm:
            print(node.position)
    """





    """
    for x in allpaths:
        print(s[x[0].position[0]][x[0].position[1]])
    """

    """
    for m in allpaths:
        current.append(m[1].position)
    """

    """
    for m in allpaths[0]:
        print("A = ",m.position)

    for m in allpaths[1]:
        print("B = ",m.position)

    for m in allpaths[2]:
        print("C = ",m.position)

    for m in allpaths[3]:
        print("D = ",m.position)

    """



    """    
    path = astar(s, startAndEnd[0][0], startAndEnd[0][1])
    print(path)
    """
    dr = [0, 0, -1, 1]
    dc = [-1, 1, 0, 0]
    i = 1

    while True:
        env.render()    # you can used this for printing the environment

        # sense
        s = env.look()

        # think
        # ...

        # act

        action = []
        target = []

        for path in allpaths:
            if len(path) > i:
                target.append(path[i].position)
                #print(path[i].position, path[i].time)
            else:
                target.append(path[len(path)-1].position)


        #print(target)
        k = 0

        for t in target:
            if (t[0] > current[k][0]):
                action.append(env.ACTION_DOWN)
                current[k][0] = current[k][0] + 1

            elif (t[0] < current[k][0]):
                action.append(env.ACTION_UP)
                current[k][0] = current[k][0] - 1


            elif (t[1] < current[k][1]):
                action.append(env.ACTION_LEFT)
                current[k][1] = current[k][1] - 1


            elif (t[1] > current[k][1]):
                action.append(env.ACTION_RIGHT)
                current[k][1] = current[k][1] + 1

            elif (t == current[k]):
                action.append(env.ACTION_WAIT)


            k = k + 1

        #print(action)
        i = i + 1
        ob, rew, done = env.step(action)


        if done:
            break

    env.close()


if __name__ == "__main__":
    run_agent()


