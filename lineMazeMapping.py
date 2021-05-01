import numpy as np
import matplotlib.pyplot as plt
import math

class edge:
    def __init__(self,node1coords, node2coords, weight):
        self.startPt = node1coords
        self.endPt = node2coords
        self.weight = weight
class node:
    def __init__(self,name, coords):
        self.name = name
        self.coords = coords
        self.Xcoord = coords[0]
        self.Ycoord = coords[1]

class mazeMap:
    def __init__(self,nodeList,edgeList):
        self.nodes = nodeList
        self.edges = edgeList

    

def visualizeMap(mazeMap):
    for node in mazeMap.nodes:
        plt.plot(node.Xcoord,node.Ycoord, color = 'blue',marker = 'o')
        plt.annotate(node.name, (node.Xcoord-1,node.Ycoord-1))
    
    for edge in mazeMap.edges:
        plt.plot([edge.startPt[0],edge.endPt[0]],[edge.startPt[1],edge.endPt[1]],color = 'blue')
    plt.axis('scaled')
    plt.show()


class robot:
    def __init__(self,maze):
        self.position =  np.array([0,0])
        self.adjList = {}
        self.currentPoint = 0
        self.heading = 2 # 1E, 2N, 3W, 4S
        self.totalPoints = 1
        self.pointArray = []
        self.typeArray = []
        self.exploredArray = []
        self.coordinateArray = []
        self.fullMap = maze
        self.exploredMap = mazeMap([[0,0]],[])
        self.moveDistance = None
        self.flag = False

    def analyzeIntersection(self):
        intersectionType = 0
        self.neighbors = []
        self.directions = []
        self.prevPoint = self.currentPoint
        print("----------------------------")
        print("Current Node: ", self.currentPoint)
        print("Position: ", self.position)
        print("Heading: ", self.heading)
        print("Coordinate Array: ", self.coordinateArray)
        print("Type Array: ", self.typeArray)
        print("Explored Array: ", self.exploredArray)
        print("Point Array: ", self.pointArray)
    
        
        # Get intersection Type (Will be based on IR Readings on Real Robot)
        for edge in self.fullMap.edges:
            #Generate number of edges leaving current position
            comparison1 = self.position == edge.startPt
            comparison2 = self.position == edge.endPt
        
            if comparison1.all():
                self.neighbors.append(edge.endPt)
                intersectionType += 1
                self.directions.append(self.getDirection(edge.endPt))
            elif comparison2.all():
                self.neighbors.append(edge.startPt)
                intersectionType += 1
                self.directions.append(self.getDirection(edge.startPt))

        #subtract one to get intersection "Type"
        intersectionType -= 1

        #If node has never been visited
        if list(self.position) not in self.coordinateArray:
            #Add new node to point array 
            self.pointArray.append(len(self.pointArray))

            #Set current point
            self.currentPoint = self.pointArray[-1]

            #If you've moved add connection to adjacency list
            if self.moveDistance != None:
                if self.prevPoint in self.adjList.keys() and [self.currentPoint, self.moveDistance] not in self.adjList[self.prevPoint]:
                    self.adjList[self.prevPoint].append([self.currentPoint, self.moveDistance])
                else:
                    self.adjList[self.prevPoint] = [[self.currentPoint, self.moveDistance]]
                if self.currentPoint in self.adjList.keys() and [self.prevPoint, self.moveDistance] not in self.adjList[self.currentPoint]:
                    self.adjList[self.currentPoint].append([self.prevPoint, self.moveDistance])

                else:
                    self.adjList[self.currentPoint] = [[self.prevPoint, self.moveDistance]]

            # Update various arrays with new node info        
            self.coordinateArray.append(list(self.position))
            self.exploredArray.append(1)
            self.typeArray.append(intersectionType)
            self.totalPoints +=1

            #Choose next point to travel
            self.chooseNextPoint()

        #Else if node hasn't been "fully explored"
        elif self.typeArray[self.coordinateArray.index(list(self.position))] - self.exploredArray[self.coordinateArray.index(list(self.position))]>0:
            #TODO: if revisiting a node
            self.currentPoint = self.coordinateArray.index(list(self.position))
           
            if [self.prevPoint, self.moveDistance] not in self.adjList[self.currentPoint]:
                self.adjList[self.currentPoint].append([self.prevPoint, self.moveDistance])
            if [self.currentPoint, self.moveDistance] not in self.adjList[self.prevPoint]:
                self.adjList[self.prevPoint].append([self.currentPoint, self.moveDistance])
            print("NOT FULLY VISITED")
            print("Times Visited: ", self.exploredArray[self.coordinateArray.index(list(self.position))])
            self.chooseNextPoint(self.exploredArray[self.coordinateArray.index(list(self.position))])
            self.exploredArray[self.coordinateArray.index(list(self.position))]+=1

        #Else node has been "Fully Explored"
        else:
            self.currentPoint = self.coordinateArray.index(list(self.position))
            if [self.prevPoint, self.moveDistance] not in self.adjList[self.currentPoint]:
                self.adjList[self.currentPoint].append([self.prevPoint, self.moveDistance])
            if [self.currentPoint, self.moveDistance] not in self.adjList[self.prevPoint]:
                self.adjList[self.prevPoint].append([self.currentPoint, self.moveDistance])
            
            #Generate unexplored Nodes
            unexplored = self.getUnexploredNodes()

            #Djikstras algorithm to generate path to nearest unfully explored node
            closestUnexploredPath = self.djikstrasNextPoint(unexplored)
            for node in closestUnexploredPath:
                self.nextPoint = np.array(self.coordinateArray[node])
                self.move()
            
            #move through path to nearest unexplored
            self.flag = True
            self.analyzeIntersection()

        print("ADJ: ", self.adjList)

    def getDirection(self,nextPoint):
        diff = self.position-nextPoint
    
        if diff[0]<0:
            direction = 1
        elif diff[0]>0:
            direction = 3
        elif diff[1]<0:
            direction = 2
        elif diff[1]>0:
            direction = 4

        return direction

    def chooseNextPoint(self,timesVisited = 0, fullyVisited = False):
        opposite = {
                1:3,
                2:4,
                3:1,
                4:2
                }

        #for direction in self.directions
        if fullyVisited == False:
            #Pick Next Direction Based on Priority List
            nextDirection = sorted(self.directions)[timesVisited]

            #Ensure if possible the robot doesn't turn 180 degrees
            if nextDirection == opposite.get(self.heading) and len(self.directions)-1>timesVisited:
                nextDirection = sorted(self.directions)[timesVisited+1] 

            #Set new Heading
            self.heading = nextDirection
        
        self.nextPoint = np.array(self.neighbors[self.directions.index(nextDirection)])
        print("NEXT PT: ", self.nextPoint)
    
    def djikstrasNextPoint(self,unexploredNodesList):
        Unexplored = self.pointArray
        explored = []
        neighbors = []
        Distances = [] 
        Paths = []
        currentNode = self.currentPoint 
        #Initialize inf distance and empty paths
        for i in range(self.totalPoints-1):
            if i == self.currentPoint:
                Distances.append(0)
                Paths.append([self.currentPoint])
            else:
                Distances.append(math.inf)
                Paths.append([])
        while len(Unexplored) > 0:
            for neighbor in self.adjList[currentNode]:
                if neighbor[0] not in explored and neighbor[0] not in neighbors:
                    neighbors.append(neighbor[0])
                    if neighbor[1] + Distances[currentNode] < Distances[neighbor[0]]:
                        Distances[neighbor[0]] = neighbor[1] + Distances[currentNode]
                        Paths[neighbor[0]] = Paths[currentNode].copy()
                        
                        Paths[neighbor[0]].extend([neighbor[0]])

            explored.append(currentNode)
            Unexplored.remove(currentNode)
        
            if len(neighbors) > 0:
                currentNode = neighbors[0]
                neighbors.remove(currentNode)

        unexploredDistances = []
        for node in unexploredNodesList:
            unexploredDistances.append(Distances[node])

        unexploredPath = Paths[unexploredNodesList[unexploredDistances.index(min(unexploredDistances))]]
        return unexploredPath
        


    def getUnexploredNodes(self):
        #Returns list of nodes that havent been fully explored
        unexploredNodes = []
        for node in self.pointArray:
            if self.typeArray[node]-self.exploredArray[node] > 0:
                 unexploredNodes.append(self.pointArray[node])

        return unexploredNodes

    def turn(self):
        pass
    
    def move(self):
        print(str(self.position) + " --> "+ str(self.nextPoint))
        self.moveDistance = max(abs(self.nextPoint-self.position))
        self.position = self.nextPoint
        self.nextPoint = []

       
    


#Generate Example Map
node1 = node('Start', np.array([0,0]))
node2 = node('A', np.array([0,10]))
node3 = node('B', np.array([6,10]))
node4 = node('C', np.array([6,13]))
node5 = node('D', np.array([10,13]))
node6 = node('E', np.array([10,18]))
node7 = node('F', np.array([13,18]))
node8 = node('G', np.array([8,18]))
node9 = node('H', np.array([8,25]))
node10 = node('I', np.array([0,25]))
node11 = node('J', np.array([0,18]))
node12 = node('K', np.array([-8,18]))
node13 = node('L', np.array([6,6]))
node14 = node('M', np.array([7,6]))
node15 = node('N', np.array([5,6]))
node16 = node('O', np.array([10,12]))


nodeList = [node1, node2, node3, node4, node5, node6, node7, node8, node9, node10, node11, node12, node13, node14, node15, node16]

edge1 = edge(node1.coords,node2.coords,10)
edge2 = edge(node2.coords,node3.coords, 6)
edge3 = edge(node3.coords,node4.coords, 3)
edge4 = edge(node4.coords,node5.coords, 4)
edge5 = edge(node5.coords,node6.coords, 5)
edge6 = edge(node6.coords,node7.coords, 3)
edge7 = edge(node6.coords,node8.coords, 2)
edge8 = edge(node8.coords,node9.coords, 7)
edge9 = edge(node9.coords,node10.coords, 8)
edge10 = edge(node10.coords,node11.coords, 7)
edge11 = edge(node11.coords,node12.coords, 8)
edge12 = edge(node11.coords,node2.coords, 8)
edge13 = edge(node8.coords,node11.coords, 8)
edge14 = edge(node3.coords,node13.coords, 4)
edge15 = edge(node13.coords,node14.coords, 1)
edge16 = edge(node13.coords,node15.coords, 1)
edge17 = edge(node5.coords,node16.coords, 1)

edgeList = [edge1, edge2, edge3, edge4,edge5, edge6, edge7, edge8,edge9, edge10, edge11, edge12, edge13, edge14, edge15, edge16, edge17]

maze = mazeMap(nodeList,edgeList)

visualizeMap(maze)

#Simulate line following robot on generated map
lineFollower = robot(maze)

while lineFollower.flag == False:
    lineFollower.analyzeIntersection()
    print(lineFollower.getUnexploredNodes())
    lineFollower.move()




