#!/usr/bin/env pybricks-micropython
import time
import math
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog

# Create your objects here.
ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
gyro_sensor = GyroSensor(Port.S1)

start = [0.305, 1.219] # Start location
goal = [3.658, 1.829]  # Goal location

MAX_OBSTACLES = 25     # Maximum number of obstacles 
num_obstacles = 13     # Number of obstacles 
obstacle_indices = []
obstacle = [
[0.61, 2.743],[0.915, 2.743],[1.219, 2.743],[1.829, 1.219],
[1.829, 1.524],[ 1.829, 1.829], [1.829, 2.134],[2.743, 0.305],
[2.743, 0.61],[2.743, 0.915],[2.743, 2.743],[3.048, 2.743],
[3.353, 2.743],
[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],[-1,-1],
[-1,-1],[-1,-1],[-1,-1]
]


# Node class used to track the order in which a path is found during A*
class Node:
    def __init__(self, index, parent, g=0, h=0):
        self.index = index # [x, y]
        self.parent = parent
        self.g = g # Cost from start
        self.h = h # Heuristic to goal
        self.f = g + h # Total cost

    # Compare nodes
    def __eq__(self, other):
        if isinstance(other, Node):
            return self.index == other.index
        return False
    
    # Sort fringe
    def __lt__(self, other):
        return self.f < other.f
    
    # Use for sets/dictionaries
    def __hash__(self):
        return hash(tuple(self.index))

    # Find & return children of current node
    def expand(self, goal_index):
        childList = []
        x, y = self.index
        
        # List of possible neighbor indices [nx, ny]
        possible_indices = []
        # Check grid boundaries (0-15 for x, 0-9 for y)
        if x > 0:  possible_indices.append([x - 1, y]) # Left
        if x < 15: possible_indices.append([x + 1, y]) # Right
        if y > 0:  possible_indices.append([x, y - 1]) # Down
        if y < 9:  possible_indices.append([x, y + 1]) # Up
        
        for idx in possible_indices:
            # Create a new node for each valid neighbor
            childList.append(Node(idx, self, self.g+1, heuristic(idx, goal_index)))
        
        return childList


# Heuristic function (Manhattan distance)
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# A* search
def graph_search():
    # Define start and goal indices from global variables
    start_index = [round(start[0]/0.305), round(start[1]/0.305)]
    goal_index = [round(goal[0]/0.305), round(goal[1]/0.305)]

    # Create start node: g=0, h=heuristic(start, goal)
    start_node = Node(start_index, None, 0, heuristic(start_index, goal_index))

    # Fringe begins with only the start node
    fringe = [start_node]
    
    # Closed set created as a set of tuples for fast lookups
    closed_set = set()

    # while fringe not empty
    while len(fringe) > 0:
        # Sort fringe to get node with lowest cost
        fringe.sort()

        # Pop node from front of list
        currentNode = fringe.pop(0)

        # Check if node is goal
        if currentNode.index == goal_index:
            return currentNode # Return goal

        # Add index to closed set
        closed_set.add(tuple(currentNode.index))

        # Expand node to get children
        childList = currentNode.expand(goal_index)
        
        for child in childList:
            # Check if child is an obstacle
            if child.index in obstacle_indices:
                continue
            
            # Check if child is in closed set
            if tuple(child.index) in closed_set:
                continue

            # Check if child is in fringe
            in_fringe = False
            for node_in_fringe in fringe:
                if node_in_fringe.index == child.index:
                    in_fringe = True
                    # If new path to child is shorter, update the node in fringe
                    if child.g < node_in_fringe.g:
                        node_in_fringe.g = child.g
                        node_in_fringe.f = child.f
                        node_in_fringe.parent = currentNode
                    break # Stop searching fringe

            # If not in fringe, add child
            if not in_fringe:
                fringe.append(child)
    
    # return start if path not found
    return Node(start, None)


def correct_left_turn():
    # NEGATIVE ANGLE CORRECTOR    
    # Calculating the angle, and the offset we must correct
    cur_ang = gyro_sensor.angle() / 90
    ang_dec, ang_whole = math.modf(cur_ang)
    print(str(ang_dec) + str(ang_whole))

    # Did not go far enough
    if abs(ang_dec) > .5:
        desired_angle = (ang_whole * 90) - 90 

    # Went to far
    elif abs(ang_dec) <= .5:
        desired_angle = (ang_whole * 90) - 0 

    else:
        print("perfect turn")

    print("Desired Angle " + str(desired_angle))
    while gyro_sensor.angle() != desired_angle:
        if gyro_sensor.angle() < desired_angle:
            while gyro_sensor.angle() < desired_angle:
                # Turn the motors LEFT really slowly until it reaches desired_angle
                print("Angle " + str(gyro_sensor.angle()))
                left_motor.run(50)
                right_motor.run(-50)
                #left_motor.run_angle(100, -1.7, wait=False)
                #right_motor.run_angle(1000, 1.7, wait=True)
            left_motor.stop()
            right_motor.stop()
            time.sleep(.01)

        else:
            while gyro_sensor.angle() > desired_angle:
                # Turn the motors RIGHT really slowly until it reaches 90
                print("Angle  " + str(gyro_sensor.angle()))
                left_motor.run(-50)
                right_motor.run(50)
                #left_motor.run_angle(100, 1.7, wait=False)
                #right_motor.run_angle(1000, -1.7, wait=True)
            left_motor.stop()
            right_motor.stop()
            time.sleep(.01)

    print("Angle has been corrected to " + str(gyro_sensor.angle()))


def correct_right_turn():
    # POSITIVE ANGLE CORRECTOR    
    # Calculating the angle, and the offset we must correct
    cur_ang = gyro_sensor.angle() / 90
    ang_dec, ang_whole = math.modf(cur_ang)
    print(str(ang_dec) + str(ang_whole))


    # Did not go far enough
    if abs(ang_dec) > .5:
        desired_angle = (ang_whole * 90) + 90

    # Went to far
    elif abs(ang_dec) <= .5:
        desired_angle = (ang_whole * 90) + 0

    else:
        print("perfect turn")

    print("Desired Angle " + str(desired_angle))

    while gyro_sensor.angle() != desired_angle:
        # If the angle is larger than 90, move back
        if gyro_sensor.angle() > desired_angle:
            while gyro_sensor.angle() > desired_angle:
                # Turn the motors LEFT really slowly until it reaches desired_angle
                print("Angle greater than " + str(desired_angle))
                left_motor.run(-100)
                right_motor.run(100)
            left_motor.stop()
            right_motor.stop()
            time.sleep(.01)

        # Else if the angle is smaller than 90, move forward
        else:
            while gyro_sensor.angle() < desired_angle:
                # Turn the motors RIGHT really slowly until it reaches 90
                print("Angle less than " + str(desired_angle))
                left_motor.run(100)
                right_motor.run(-100)
            left_motor.stop()
            right_motor.stop()
            time.sleep(.01)
    
    print("Angle has been corrected to " + str(gyro_sensor.angle()))


def angle_correction():
    if gyro_sensor.angle() < 0:
        correct_left_turn()
    
    elif gyro_sensor.angle() > 0:
        correct_right_turn()
    time.sleep(1)


def move_forward(distance):
    dgpd = 360 / (.1725 * .99) 
    right_motor.run_angle(200, distance*dgpd, wait=False)
    left_motor.run_angle(200, distance*dgpd, wait=True)
    time.sleep(1)


def right_turn():
    # TURN 90 DEGREES TO THE RIGHT
    curAngle = gyro_sensor.angle() 
    
    while gyro_sensor.angle() < (curAngle + 90):
        print("Gyro Sensor Angle " + str(gyro_sensor.angle()))
        print("Right Motor Angle " + str(right_motor.angle()))
        left_motor.run(100)
        right_motor.run(-100)
    left_motor.stop()
    right_motor.stop()
    time.sleep(1)


def left_turn():
    # TURN 90 DEGREES TO THE LEFT
    curAngle = gyro_sensor.angle() 

    while gyro_sensor.angle() > (curAngle - 90):
       print("Gyro Sensor Angle " + str(gyro_sensor.angle()))
       print("Right Motor Angle " + str(right_motor.angle()))
       left_motor.run(-100)
       right_motor.run(100)
    left_motor.stop()
    right_motor.stop()
    time.sleep(1)


# Work in progress, but this should return the robot to its original orientation, i.e., the starting angle
def original_orientation(org_angle):
    curAngle = gyro_sensor.angle() 
    delta_angle = (curAngle - org_angle)
    # If change in angle is negative, turn right
    if delta_angle < 0:
        # while loop until angle is 0
        while (delta_angle < 0):
            right_turn()
            # This could be done differently, probably should involve the current angle
            delta_angle = delta_angle + 90

    # If change in angle is positive, turn left
    else:
        # while loop until angle is 0
        while (delta_angle > 0):
            left_turn()
            # This could be done differently, probably should involve the current angle
            delta_angle = delta_angle - 90


def main():
    # This program requires LEGO EV3 MicroPython v2.0 or higher.
    # Click "Open user guide" on the EV3 extension tab for more information.

    # create workspace and populate with obstacles
    workspace = [[0]*16]*10

    # populate obstacle indices
    for idx in range(0,num_obstacles):
        x = round((obstacle[idx][0])/0.305)
        y = round((obstacle[idx][1])/0.305)

        obstacle_indices.append([x,y])
        obstacle_indices.append([x-1,y])
        obstacle_indices.append([x,y-1])
        obstacle_indices.append([x-1,y-1])

    # populate workspace costs
    for y in range(0,10):
        for x in range(0,16):
            if([x,y] in obstacle_indices):
                workspace[y][x] = 1000
            else:
                workspace[y][x] = (abs(x - goal[0]) + abs(y - goal[1]))
    

    # Run A* to find path to goal
    goalNode = graph_search()
    found_path = []

    # place path to goal into found_path
    while(goalNode.parent != None):
        found_path.insert(0,goalNode.index)
        goalNode = goalNode.parent
    found_path.insert(0,goalNode.index)

    # If the found_path is the given start, exit
    if(found_path[0] == start):
        print("No path to the goal exists.")
        return

    # trim path by only including indices for the start, end, and turns
    simplified_path = [found_path[0]]
    for step1 in range(1,len(found_path)):
        if(found_path[step1][0] == found_path[step1-1][0]):
            for step2 in range(step1,len(found_path)):
                if(found_path[step2][0] != found_path[step2-1][0]):
                    if(found_path[step2-1] not in simplified_path):
                        simplified_path.append(found_path[step2-1])
                    break
        elif(found_path[step1][1] == found_path[step1-1][1]):
            for step2 in range(step1,len(found_path)):
                if(found_path[step2][1] != found_path[step2-1][1]):
                    if(found_path[step2-1] not in simplified_path):
                        simplified_path.append(found_path[step2-1])
                    break  
        if( (step1 == len(found_path)-1) and (found_path[step1] not in simplified_path) ):
            simplified_path.append(found_path[step1])

    print("Pathfinding Information:")
    for item in found_path:
        print(item)
    print()
    for item in simplified_path:
        print(item)
    print()

    # # setup variables used for optometry, angle 0 is to the world-frame direction of right
    # current_location = [simplified_path[0][0]*0.305,simplified_path[0][1]*0.305]
    # current_orientation = gyro_sensor.angle()
    # desired_distance = 0
    # desired_angle = 0
    # print("Location: ",current_location)
    # print("Orientation: ",current_orientation)

    # # move robot across the path
    # for move in range(1,len(simplified_path)):
    #     if(found_path[move][0] == found_path[move-1][0]):
    #         desired_distance = found_path[move][1] - found_path[move-1][1]
    #         if(desired_distance > 0): # move right
    #             if(current_orientation != 0):
    #                 if(current_orientation > 0):
    #                     right_turn()
    #                     correct_right_turn()
    #                 else:
    #                     left_turn()
    #                     correct_left_turn()
    #             move_forward(desired_distance)
    #         else: # move left
    #             if(current_orientation != 180):
    #                 if(current_orientation > 180):
    #                     right_turn()
    #                     correct_right_turn()
    #                 else:
    #                     left_turn()
    #                     correct_left_turn()
    #             move_forward(-desired_distance)
    #     elif(found_path[move][1] == found_path[move-1][1]):
    #         desired_distance = found_path[move][0] - found_path[move-1][0]
    #         if(desired_distance > 0): # move up
    #             if(current_orientation != 90):
    #                 if(current_orientation > 90):
    #                     right_turn()
    #                     correct_right_turn()
    #                 else:
    #                     left_turn()
    #                     correct_left_turn()
    #             move_forward(desired_distance)
    #         else: # move down
    #             if(current_orientation != 270):
    #                 if(current_orientation > 270):
    #                     right_turn()
    #                     correct_right_turn()
    #                 else:
    #                     left_turn()
    #                     correct_left_turn()
    #             move_forward(-desired_distance)


main()