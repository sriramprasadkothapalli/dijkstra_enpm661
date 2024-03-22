## Dijstra Algorithm for path planning
README for Dijkstra's Algorithm Pathfinding
This project implements Dijkstra's algorithm to find the shortest path
between two points in a given obstacle-laden environment. The algorithm
visualizes the pathfinding process and the final path.
Dependencies
To run this code, you will need Python installed on your system along with
the following libraries:
```
NumPy
Matplotlib
OpenCV-Python
Shapely (for defining hexagon as obstacle in the region)
```
You can install these dependencies via pip:
```
pip install numpy matplotlib opencv-python shapely
```
Running the Code
Open a Terminal or Command Prompt.
Navigate to the directory containing the script. If your script is in a
folder called "Dijkstra_Pathfinding" on your desktop, you would use a
command like:
```
cd Desktop/Dijkstra_Pathfinding
```
Providing Inputs
After running the script, you will be prompted to enter the start and goal
coordinates in the terminal or command prompt. Here's the format you
should follow:
```
Enter x co-ordinate of start position: Enter the X-coordinate of the start
position and press Enter.
Enter y co-ordinate of start position: Enter the Y-coordinate of the start
position and press Enter.
Enter x co-ordinate of goal position: Enter the X-coordinate of the goal
position and press Enter.
Enter y co-ordinate of goal position: Enter the Y-coordinate of the goal
position and press Enter.
```
EXAMPLE Input
```
Enter x co-ordinate of start position: 10
Enter y co-ordinate of start position: 30
Enter x co-ordinate of goal position: 750
Enter y co-ordinate of goal position: 100
```
OUTPUT
The program visualizes the exploration process and the final path in a
video file named dijkstra_viz.avi.
It also saves an image named Path_Taken.jpg showing the start point, goal
point, obstacles, and the path taken.
