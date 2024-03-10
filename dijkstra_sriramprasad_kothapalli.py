
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
    elif (x >= 895 and x <= 1105 and y >= 45 and y <= 130 ):
        return True
    
    elif (x >= 895 and x <= 1105 and y <= 455  and y >= 370):
        return True
    
    elif (x <= 1095 and x >= 1015 and y <= 375  and y >= 125):
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

# function to backtrack to generate path
## input: pointer to goal node
## output: list of path taken

def backtrack(current):
    path = []
    parent = current
    while parent!= None:
        path.append(parent.loc)
        parent = parent.parent
    return path[::-1]


def dijkstra_algo(start, goal):
    start_time = time.time()
    # create object to save vizualization video
    saveViz = cv2.VideoWriter('dijkstra_viz.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 1000, (1200,500))
    region = drawObstacles((1200,500))
    space_viz = drawObstacles((1200,500))
    pathTaken = []
    
    # check if start position is in obstacle region
    if ObstacleSpace(start):
        print("Start position is in Obstacle region")
        return False, pathTaken
    
    # check if goal position is in obstacle region
    if ObstacleSpace(goal):
        print("Goal position is in Obstacle region")
        return False, pathTaken
    
    # mark start and goal locations
    cv2.circle(region, (start[0],region.shape[0]-start[1]-1), 1, (0, 0, 255), 5)
    cv2.circle(region, (goal[0],region.shape[0]-goal[1]-1), 1, (255, 0, 0), 5)
    #changed
    # initialize open and closed list
    openList = []
    openListSet = set()  # This set keeps track of the locations (x,y) in the open list
    heapq.heappush(openList, (0, createNode(start, None, 0)))  # Initialize with the start node
    openListSet.add(start)  # Add start location to the set
    closedList = set()  # Closed list to keep track of visited nodes
    
    while openList:
        currentCost, current = heapq.heappop(openList)
        openListSet.remove(current.loc)  # Remove current node from set when popped from queue
    
        if current.loc in closedList:
            continue
        
        closedList.add(current.loc)
        # Mark the current node as visited
        if current.loc == goal:
            cv2.circle(region, (start[0], region.shape[0]-start[1]-1), 1, (0, 0, 255), 2)
            cv2.circle(region, (goal[0], region.shape[0]-goal[1]-1), 1, (255, 0, 0), 2)
            saveViz.write(region)
        # Generate path and save visualization    
            pathTaken = backtrack(current)
            # Define path color and thickness
            path_color = (0, 0, 255)  # Red color in BGR format
            path_thickness = 2  # Specify the thickness here
            for i in range(1, len(pathTaken)):
                # Get start and end points for each segment of the path
                pt1 = pathTaken[i - 1]
                pt2 = pathTaken[i]
                
                # Convert points for OpenCV format (remember to invert y-axis for OpenCV images)
                pt1_cv = (pt1[0], region.shape[0] - pt1[1] - 1)
                pt2_cv = (pt2[0], region.shape[0] - pt2[1] - 1)
                
                # Draw the line segment on the region and space_viz images
                cv2.line(region, pt1_cv, pt2_cv, path_color, path_thickness)
                cv2.line(space_viz, pt1_cv, pt2_cv, path_color, path_thickness)
                saveViz.write(region)
            # Save the visualization of the path taken
            cv2.circle(space_viz, (start[0], region.shape[0]-start[1]-1), 1, (0, 0, 255), 2)
            cv2.circle(space_viz, (goal[0], region.shape[0]-goal[1]-1), 1, (255, 0, 0), 2)
            cv2.imwrite("Path_Taken.jpg", space_viz)
            saveViz.release()
            end_time = time.time()
            print("Path found with total cost:", current.cost_to_come)
            print("Time taken = ", (end_time - start_time) / 60, " min")
            return True, pathTaken
    
        for actionCost, actionChild in getChildList(current):
                 if actionChild.loc not in closedList and actionChild.loc not in openListSet:
                    actionChild.cost_to_come = current.cost_to_come + actionCost
                    heapq.heappush(openList, (actionChild.cost_to_come, actionChild))
                    openListSet.add(actionChild.loc)  # Add new node location to the set
                    x, y = actionChild.loc
                    region[region.shape[0]-y-1, x] = [0, 255, 255]
                    #resized_frame = cv2.resize(region, (300, 125))  
                    saveViz.write(region)
                    
    return False, []
        
# main function to take user input and call dijkstra function
def main():
    print("----Dijkstra's Algorithm----")
    # Initialize start and goal coordinates
    start_x = start_y = goal_x = goal_y = None
        # Loop to ensure start coordinates are within bounds
    while True:
        try:
            start_x = int(input("Enter x co-ordinate of start position: "))
            start_y = int(input("Enter y co-ordinate of start position: "))
            if start_x < 0 or start_x > 1200 or start_y < 0 or start_y > 500:
                print("Start position must be within a 1200x500 region. Please enter values again.")
            else:
                break
        except ValueError:
            print("Please enter valid integer values.")
    
    # Loop to ensure goal coordinates are within bounds
    while True:
        try:
            goal_x = int(input("Enter x co-ordinate of goal position: "))
            goal_y = int(input("Enter y co-ordinate of goal position: "))    
            if goal_x < 0 or goal_x > 1200 or goal_y < 0 or goal_y > 500:
                print("Goal position must be within a 1200x500 region. Please enter values again.")
            else:
                break
        except ValueError:
            print("Please enter valid integer values.")
    
    start = (start_x, start_y)
    goal = (goal_x, goal_y)
    
    # Call dijkstra_algo function to find optimal path
    success, backPath = dijkstra_algo(start, goal)
    if success:
        print("Path generated")
    else:
        print("Path not found")


if __name__ == "__main__":
    # main function
    main()