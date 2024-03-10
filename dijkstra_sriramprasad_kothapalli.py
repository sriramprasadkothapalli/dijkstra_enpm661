
# import required libraries

import numpy as np
import matplotlib.pyplot as plt
import cv2
import time
import heapq
from shapely.geometry import Polygon
from shapely.geometry import Point


# function to draw obstacle region as given

global bloated_obstacles
bloated_obstacles = []
def drawObstacles(size = (1200,500)):
    w,h = size

    
    # initialize empty region
    region = np.zeros((h+1, w+1, 3), dtype = np.uint8) 
    region.fill(255) # Fill region with white color
    # draw obstacles
    cv2.rectangle(region, (100, 500), (175, 100), (255, 0, 255), 5 )
    cv2.rectangle(region, (100, 500), (175, 100), (0, 0, 255), -1)

    cv2.rectangle(region, (100+75+100, 400), (100+75+100+75, 0), (255, 0, 255), 5)
    cv2.rectangle(region, (100+75+100, 400), (100+75+100+75, 0), (0, 0, 255), -1)
    
    hexagon_vertices = np.array([[771, 320], [650, 390], [528, 320], [528, 180] , [650, 110], [771, 179]])
    # Create a Shapely Polygon from your vertices
    hexagon = Polygon(hexagon_vertices)

# Bloat the hexagon by a defined margin (for example, 5 units)
    bloated_hexagon = hexagon.buffer(5)  # Adjust the 5 units to your specific needs
    bloated_obstacles.append(bloated_hexagon) 

    u_shape = np.array([[1200-100-200,450],[1200-100,450],[1200-100, 50],[1200-100-200,50],[1200-100-200,125],[1200-100-200+120,125],[1200-100-200+120,375],[1200-100-200,375]])

    hexagon_for_cv = np.array(hexagon.exterior.coords).reshape((-1, 1, 2)).astype(np.int32)
    # fill the shapes 
    region = cv2.fillPoly(region, pts=[hexagon_for_cv,u_shape], color=(0, 0, 255))
    region = cv2.flip(region,0)
    return region

# function to check if robot is in obstacle region, also bloats the walls and the obstacles
## input: location of robot
## output: True if robot is in obstacle region, False otherwise

def ObstacleSpace(location):
   
    xMax, yMax = [1200+1,500+1]
    xMin, yMin = [0,0]
    x,y = location
    point = Point(x, y)  # Create a Shapely Point for the given location
    bl = 5
    
#for walls 
    if (x < xMin+bl) or (y < yMin+bl) or (x >= xMax-bl) or (y >= yMax-bl):
        return True
# for rectangles
    elif (x <= 175+bl) and (x >= 100-bl) and (y < 500) and (y >= 100-bl):
        return True
    elif (x <= 350+bl)  and (x >= 275-bl) and (y <= 400+bl) and (y > 0):
        return True
# for u-shape 
    elif (x >= 895 and x <= 1105 and y >= 45 and y <= 130  or x >= 1015 and x <= 1100 and y >= 45 and y <= 180):
        return True
    
    elif (x >= 895 and x <= 1105 and y <= 455  and y >= 370):
        return True

   # Check global bloated obstacles
    for obstacle in bloated_obstacles:
        if obstacle.contains(point):
            return True  # The point is inside an obstacle

    else:
        return False
    

# create node class to initialise each node with location, parent and cost-to-come

class createNode:
    def __init__(self, loc, parent, cost_to_come):
        self.loc = loc
        self.parent = parent
        self.cost_to_come = cost_to_come

# Comparison method to enable direct comparisons between createNode instances within heapq operations
    def __lt__(self, other):
        return self.cost_to_come < other.cost_to_come

# create action set for the robot, creates a node in the direction for possibility
## input: pointer to current node
## output: cost of action, created child


# action for moving down
def moveDown(node):
    cost = 1
    x,y = node.loc
    actionChild = createNode((x,y-1),node,node.cost_to_come+cost)
    return cost,actionChild

# action for moving up
def moveUp(node):
    cost = 1
    x,y = node.loc
    actionChild = createNode((x,y+1),node,node.cost_to_come+cost)
    return cost,actionChild

# action for moving left
def moveLeft(node):
    cost = 1
    x,y = node.loc
    actionChild = createNode((x-1,y),node,node.cost_to_come+cost)
    return cost,actionChild

# action for moving right
def moveRight(node):
    cost = 1
    x,y = node.loc
    actionChild = createNode((x+1,y),node,node.cost_to_come+cost)
    return cost,actionChild

# action for moving Up-left
def moveUpLeft(node):
    cost = 1.4
    x,y = node.loc
    actionChild = createNode((x-1,y+1),node,node.cost_to_come+cost)
    return cost,actionChild

# action for moving Up-right
def moveUpRight(node):
    cost = 1.4
    x,y = node.loc
    actionChild = createNode((x+1,y+1),node,node.cost_to_come+cost)
    return cost,actionChild

# action for moving Down-left
def moveDownLeft(node):
    cost = 1.4
    x,y = node.loc
    actionChild = createNode((x-1,y-1),node,node.cost_to_come+cost)
    return cost,actionChild

# action for moving Down-right
def moveDownRight(node):
    cost = 1.4
    x,y = node.loc
    actionChild = createNode((x+1,y-1),node,node.cost_to_come+cost)
    return cost,actionChild

# function to generate all physically valid children
## input: pointer to current node
## output: Valid children list of a node

def getChildList(node):
    xMax, yMax = [1200+1,500+1]
    xMin, yMin = [0,0]
    
    x_cur, y_cur = node.loc
    children = [] 
    
    # check if moving down is possible 
    if y_cur > yMin:
        (actionCost, child) = moveDown(node)
        if not ObstacleSpace(child.loc):
            # if node is not in obstacle region, append in child list
            children.append((actionCost, child))
        else:
            del child
            
    # check if moving up is possible 
    if y_cur < yMax:
        (actionCost, child) = moveUp(node)
        if not ObstacleSpace(child.loc):
            # if node is not in obstacle region, append in child list
            children.append((actionCost, child))
        else:
            del child
            
    # check if moving left is possible 
    if x_cur > xMin:
        (actionCost, child) = moveLeft(node)
        if not ObstacleSpace(child.loc):
            # if node is not in obstacle region, append in child list
            children.append((actionCost, child))
        else:
            del child
    
    # check if moving right is possible 
    if x_cur < xMax:
        (actionCost, child) = moveRight(node)
        if not ObstacleSpace(child.loc):
            # if node is not in obstacle region, append in child list
            children.append((actionCost, child))
        else:
            del child  
    
    # check if moving up-right is possible 
    if y_cur < yMax and x_cur < xMax:
        (actionCost, child) = moveUpRight(node)
        if not ObstacleSpace(child.loc):
            # if node is not in obstacle region, append in child list
            children.append((actionCost, child))
        else:
            del child
    
    
    # check if moving up-left is possible 
    if y_cur < yMax and x_cur > xMin:
        (actionCost, child) = moveUpLeft(node)
        if not ObstacleSpace(child.loc):
            # if node is not in obstacle region, append in child list
            children.append((actionCost, child))
        else:
            del child
    
    
    # check if moving down-right is possible 
    if y_cur > yMin and x_cur < xMax:
        (actionCost, child) = moveDownRight(node)
        if not ObstacleSpace(child.loc):
            # if node is not in obstacle region, append in child list
            children.append((actionCost, child))
        else:
            del child
    
    
    # check if moving down-left is possible 
    if y_cur > yMin and x_cur > xMin:
        (actionCost, child) = moveDownLeft(node)
        if not ObstacleSpace(child.loc):
            # if node is not in obstacle region, append in child list
            children.append((actionCost, child))
        else:
            del child
    
    return children

# function to generate itme for open list
## input: pointer to current node
## output: tuple of (node.cost_to_come, node)

def olItem(node):
    item = (node.cost_to_come, node)
    return item




if __name__ == "__main__":
    # main function
    main()