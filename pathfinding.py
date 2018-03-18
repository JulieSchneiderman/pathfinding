from functools import total_ordering

class Graph:
    def __init__(self):
        return
    
    def cost(self):
        return 1
    
    def neighbors(self, currentNode, numNeighbors):
        if (numNeighbors == 4):
            results = [Node() for _ in range(numNeighbors)]
            
            newnode = Node()
            newnode.setX(currentNode.getX()) 
            newnode.setY(currentNode.getY()+1)  #down
            results[0] = newnode
            
            newnode = Node()
            newnode.setX(currentNode.getX())
            newnode.setY(currentNode.getY()-1) #up
            results[1] = newnode
            
            newnode = Node()
            newnode.setX(currentNode.getX()+1) #right
            newnode.setY(currentNode.getY()) 
            results[2] = newnode
            
            newnode = Node()
            newnode.setX(currentNode.getX()-1) #left
            newnode.setY(currentNode.getY()) 
            results[3] = newnode

        elif (numNeighbors == 8):
            results = [Node() for _ in range(numNeighbors)]
            
            newnode = Node()
            newnode.setX(currentNode.getX()) 
            newnode.setY(currentNode.getY()+1)  #down
            results[0] = newnode
            
            newnode = Node()
            newnode.setX(currentNode.getX())
            newnode.setY(currentNode.getY()-1) #up
            results[1] = newnode
            
            newnode = Node()
            newnode.setX(currentNode.getX()+1) #right
            newnode.setY(currentNode.getY()) 
            results[2] = newnode
            
            newnode = Node()
            newnode.setX(currentNode.getX()-1) #left
            newnode.setY(currentNode.getY()) 
            results[3] = newnode

            newnode = Node()
            newnode.setX(currentNode.getX()+1) #up right
            newnode.setY(currentNode.getY()+1)
            results[4] = newnode

            newnode = Node()
            newnode.setX(currentNode.getX()-1) #up left
            newnode.setY(currentNode.getY()+1)
            results[5] = newnode

            newnode = Node()
            newnode.setX(currentNode.getX()+1) #down right
            newnode.setY(currentNode.getY()-1)
            results[6] = newnode

            newnode = Node()
            newnode.setX(currentNode.getX()-1) #down left
            newnode.setY(currentNode.getY()-1)
            results[7] = newnode
            
        return results

    
@total_ordering
class Node:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.Priority = 0
        return 
    
    def setX(self,xVal):
        self.x = xVal
    
    def setY(self,yVal):
        self.y = yVal
        
    def priority():
        return 
    
    def getX(self):
        return self.x 
    
    def getY(self):
        return self.y
    
    def setPriority(self, pVal):
        self.Priority = pVal
    
    def getPriority(self):
        return self.Priority
    
    def __hash__(self):
        hashfunc = 17
        hashfunc = hashfunc * 486187739 + self.x.__hash__()
        hashfunc = hashfunc * 486187739 + self.y.__hash__()
        return hashfunc
    
    def __eq__(self, obj):
        if(obj == None):
            return False
        elif self is obj:
            return True
        elif self.x == obj.x and self.y == obj.y:
            return True
        return False
    
    def __lt__(self, other):
        return (self.x, self.y) < (other.x, other.y)
    
def startGoalStates(gridstring,numNeighbors):
    grid = []
    start = Node()
    goal = Node()
    n = len(gridstring)
    
    s = gridstring[0] #first row
    
#    build grid 
    for i in range(n):
        grid.append([]) 
        for j in range(len(s)): 
            grid[i].append(0) 
 
    for row in range(n): 
        s = gridstring[row]
        for col in range(len(s)):  
            if s[col] == 'S':
                grid[row][col] = 1 #identifier value for start
                start.setX(col)
                start.setY(row)
            elif s[col] == 'G':
                grid[row][col] = 2 #identifier value for goal
                goal.setX(col)
                goal.setY(row)
            elif s[col] == 'X': 
                grid[row][col] = -1
            elif s[col] == '_':
                grid[row][col] = 0
                
    startcopy = start
    goalcopy = goal
    allinfo = [grid,startcopy,goalcopy]
    return allinfo


def greedy_search(grid,start,goal,numNeighbors):
    graph = Graph()
    start.setPriority(1)
    goal.setPriority(-1)
    frontier = [Node()]
    frontier[0] = start
    came_from = {}     
    came_from[start] = None
    current = Node()
    
    while len(frontier) != 0:
        current = frontier[0]
        frontier.remove(current)
        
        if (current.getX() ==  goal.getX()) and (current.getY() == goal.getY()):
            return reconstruct_path(grid, came_from, current, start, goal)

        for nextnode in graph.neighbors(current,numNeighbors): #list of nodes
            if nextnode not in came_from:
                if (numNeighbors == 4):
                    priority = heuristicFour(grid, goal, nextnode)
                    nextnode.setPriority(priority)
                    frontier.append(nextnode)
                    
                if (numNeighbors == 8):
                    priority = heuristicEight(grid, goal, nextnode)
                    nextnode.setPriority(priority)
                    frontier.append(nextnode)
                    
#               frontier insertion sort
                i = 1
                for i in range(len(frontier)):
                    j = i
                    while j > 0 and frontier[j -1].getPriority() > frontier[j].getPriority():
                        temp = frontier[j]
                        frontier[j] = frontier[j-1]
                        frontier[j-1] = temp
                        j-=1
                        
                came_from[nextnode] = current
            
def a_star_search(grid, start, goal,numNeighbors):
    graph = Graph()
    start.setPriority(1)
    goal.setPriority(-1)
    frontier = [Node()]
    frontier[0] = start
    came_from = {}    
    cost_so_far = {} 
    came_from[start] = None
    cost_so_far[start] = 0
    current = Node()
    
    while len(frontier) != 0:
        current = frontier[0]
        frontier.remove(current)
        
        if (current.getX() ==  goal.getX()) and (current.getY() == goal.getY()):
            return reconstruct_path(grid, came_from, current, start, goal)

        for nextnode in graph.neighbors(current,numNeighbors): #list of nodes
            new_cost = cost_so_far[current] + graph.cost()
            if nextnode not in cost_so_far or new_cost < cost_so_far[nextnode]:
                if (numNeighbors == 4):
                    cost_so_far[nextnode] = new_cost
                    priority = new_cost + heuristicFour(grid, goal, nextnode)
                    nextnode.setPriority(priority)
                    frontier.append(nextnode)
                elif(numNeighbors == 8):
                    cost_so_far[nextnode] = new_cost
                    priority = new_cost + heuristicEight(grid, goal, nextnode)
                    nextnode.setPriority(priority)
                    frontier.append(nextnode)

#               frontier insertion sort
                i = 1
                for i in range(len(frontier)):
                    j = i
                    while j > 0 and frontier[j -1].getPriority() > frontier[j].getPriority():
                        temp = frontier[j]
                        frontier[j] = frontier[j-1]
                        frontier[j-1] = temp
                        j-=1
                came_from[nextnode] = current

def heuristicFour(grid, goal, node): 
    try:
        if(grid[node.getY()][node.getX()] == -1):
            return 1000000000
    except Exception as inst:
            return 1000000000
    else:
        return abs(goal.getX() - node.getX()) + abs(goal.getY() - node.getY())

def heuristicEight(grid, goal, node):
    if(grid[node.getY()][node.getX()] == -1):
        return 1000000000
    else:
        return max(abs(node.getX() - goal.getX()),abs(node.getY() - goal.getY()))

def reconstruct_path(grid, came_from, current, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]   
        grid[current.getY()][current.getX()] = 'P'
        path.append(current)
    grid[start.getY()][start.getX()] = 'S'

    for row in range(len(grid)):
        s = grid[row]
        for col in range(len(s)):
            if s[col] == -1:
                grid[row][col] = 'X'
            elif s[col] == 0:
                grid[row][col] = '_'
            elif s[col] == 2:
                grid[row][col] = 'G'

    path.append(start)
    path.reverse()
    return grid    

def main():
    outfileA = open("pathfinding_a_out.txt",'w')
    with open('pathfinding_a.txt') as f:
        gridarray = f.readlines()
        gridarray = [x.strip() for x in gridarray]

    spaces = [] 
    for i in range(len(gridarray)):
        if gridarray[i] == '':
            spaces.append(i)
    start = 0
    for i in range(len(spaces)):
        end = spaces[i]
        newarr = gridarray[start:end]
        gridInfoArr = startGoalStates(newarr,4)
        greedy = greedy_search(gridInfoArr[0],gridInfoArr[1],gridInfoArr[2],4)
        gridInfoArr2 = startGoalStates(newarr,4)
        astar = a_star_search(gridInfoArr2[0],gridInfoArr2[1],gridInfoArr2[2],4)
        outfileA.write("Greedy:\n")
        for i in range(len(greedy)):
            flatgreedy = ''.join(greedy[i])
            outfileA.write(flatgreedy + "\n")
        outfileA.write("A*:\n")
        for i in range(len(astar)):
            flatstar = ''.join(astar[i])
            outfileA.write(flatstar + "\n")
        start = end + 1
        outfileA.write("\n")

    lastindex = len(gridarray)
    lastarr = gridarray[start:lastindex]
    gridInfoArr = startGoalStates(lastarr,4)
    lastGreedy = greedy_search(gridInfoArr[0],gridInfoArr[1],gridInfoArr[2],4)
    gridInfoArr2 = startGoalStates(lastarr,4)
    lastAstar = a_star_search(gridInfoArr2[0],gridInfoArr2[1],gridInfoArr2[2],4)

    outfileA.write("Greedy:\n")
    for i in range(len(lastGreedy)):
        flatg = ''.join(lastGreedy[i])
        outfileA.write(flatg + "\n")
    outfileA.write("A*:\n")         
    for i in range(len(lastAstar)):
        flata = ''.join(lastAstar[i])
        outfileA.write(flata + "\n")

    outfileA.write("\n")
    outfileA.close()
    #part b
    mainB()
    
    
    #part b
def mainB():
    outfileA = open("pathfinding_b_out.txt",'w')
    with open('pathfinding_b.txt') as f:
        gridarray = f.readlines()
        gridarray = [x.strip() for x in gridarray]

    spaces = [] 
    for i in range(len(gridarray)):
        if gridarray[i] == '':
            spaces.append(i)
    start = 0
    for i in range(len(spaces)):
        end = spaces[i]
        newarr = gridarray[start:end]
        gridInfoArr = startGoalStates(newarr,4)
        greedy = greedy_search(gridInfoArr[0],gridInfoArr[1],gridInfoArr[2],8)
        gridInfoArr2 = startGoalStates(newarr,4)
        astar = a_star_search(gridInfoArr2[0],gridInfoArr2[1],gridInfoArr2[2],8)

        outfileA.write("Greedy:\n")
        for i in range(len(greedy)):
            flatgreedy = ''.join(greedy[i])
            outfileA.write(flatgreedy + "\n")

        outfileA.write("A*:\n")
        for i in range(len(astar)):
            flatstar = ''.join(astar[i])
            outfileA.write(flatstar + "\n")
        start = end + 1
        outfileA.write("\n")

    lastindex = len(gridarray)
    lastarr = gridarray[start:lastindex]
    gridInfoArr = startGoalStates(lastarr,4)
    lastGreedy = greedy_search(gridInfoArr[0],gridInfoArr[1],gridInfoArr[2],8)
    gridInfoArr2 = startGoalStates(lastarr,4)
    lastAstar = a_star_search(gridInfoArr2[0],gridInfoArr2[1],gridInfoArr2[2],8)

    for i in range(len(lastGreedy)):
        flatg = ''.join(lastGreedy[i])
        outfileA.write(flatg + "\n")
    outfileA.write("A*:\n")         
    for i in range(len(lastAstar)):
        flata = ''.join(lastAstar[i])
        outfileA.write(flata + "\n")

    outfileA.close()
    
main()
    
    
    
    
    
