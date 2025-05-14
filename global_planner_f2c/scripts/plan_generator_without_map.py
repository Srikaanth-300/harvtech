#!/usr/bin/env python3
import csv
import os
import fields2cover as f2c

file_path = csv_file = os.path.expanduser("~/harvtech_ws/src/navigation/csv_files/world_xy_values/simulated_boundary.csv")    # Read the boundary points
boundary = f2c.VectorPoint()   

# Fill the boundary vector with boundary points
with open(file_path, 'r') as file:
    reader = csv.reader(file, delimiter = ',')
    for line in reader:
        x = float(line[0])
        y = float(line[1])
        boundary.append(f2c.Point(x, y))

# Define a Cells and Cells with those boundaries
cell = f2c.Cell(f2c.LinearRing(boundary)) 
cells = f2c.Cells(cell)
print(type(cells))

# Define the robot dimensions like its width and the tool width
robot = f2c.Robot(0.4824, 1.0)
r_w = robot.getCovWidth()

# Define the headland, objective (field_coverage), algorithm to generate swaths (brute_force)
const_hl = f2c.HG_Const_gen()
bf = f2c.SG_BruteForce()
obj = f2c.OBJ_FieldCoverage()


# Generate the headland and swaths 
mid_hl = const_hl.generateHeadlands(cells, 3.0 * robot.getWidth())    # Thrice the robot's width is the width of headland
swaths = bf.generateBestSwaths(obj, r_w, mid_hl.getGeometry(0))
print(type (swaths))

# Define the algorithm to order the swaths
snake_sorter = f2c.RP_Snake()    # Snake pattern is used
swaths = snake_sorter.genSortedSwaths(swaths)

# Set out parameters like what type of curve to use while changing from one swath to another, and the minimum turning radius (0.1 m is used to get smooth curvature). Dubins curve is used since we don't want to reverse the robot while changing swaths.
robot.setMinTurningRadius(0.1)
path_planner = f2c.PP_PathPlanning()
dubins = f2c.PP_DubinsCurves()
path = path_planner.planPath(robot, swaths, dubins)
discretize_path = path.discretizeSwath(0.1)
print(discretize_path.size())

# Visualization and saving the files.
f2c.Visualizer.figure()
f2c.Visualizer.plot(cells)
f2c.Visualizer.plot(mid_hl)
f2c.Visualizer.plot(swaths)
f2c.Visualizer.plot(discretize_path)
f2c.Visualizer.save("simulated_boundary_f2c_1.png")
discretize_path.saveToFile("simulated_boundary_f2c_1.csv")
