# Changes Working Directory to be at the root of FAIRIS-Lite
from collections import namedtuple
import dis
import math
import os
from turtle import distance
os.chdir("../..")

# Import MyRobot Class
from WebotsSim.libraries.MyRobot import MyRobot
from WebotsSim.libraries.MyRobot import PIDControl

# Create the robot instance.
robot = MyRobot()
cellInfo = namedtuple('tileInfo', 'x y t l u r')        #top left under right
point = namedtuple('Point', 'x y')

LIDAR_FRONT = 400
LIDAR_RIGHT = 600
LIDAR_BACK = 0     #or 799
LIDAR_LEFT = 200

FRONT_LIMIT = 0.1
SIDE_LIMIT = 0.4
TILE_TRAVEL_ERROR = 0.0001
TRAVEL_SPEED = 5
MAP_WRITEPATH = "C:/Users/AnotherGuy/FAIRIS-Lite/WebotsSim/controllers/REU_MazeData.txt"

tilePID = PIDControl(TILE_TRAVEL_ERROR, 1, 10)

visitedCells = {        #Point-boolean dictionary       (x = [-5, 6], y = [-6, 5])
    (-5, -6) : False,
    (-5, -5) : False,
    (-5, -4) : False,
    (-5, -3) : False,
    (-5, -2) : False,
    (-5, -1) : False,
    (-5, 0) : False,
    (-5, 1) : False,
    (-5, 2) : False,
    (-5, 3) : False,
    (-5, 4) : False,
    (-5, 5) : False,
    
    (-4, -6) : False,
    #(-4, -5) : False,
    (-4, -4) : False,
    #(-4, -3) : False,
    #(-4, -2) : False,
    (-4, -1) : False,
    (-4, 0) : False,
    #(-4, 1) : False,
    (-4, 2) : False,
    (-4, 3) : False,
    #(-4, 4) : False,
    (-4, 5) : False,
    
    (-3, -6) : False,
    #(-3, -5) : False,
    (-3, -4) : False,
    #(-3, -3) : False,
    #(-3, -2) : False,
    (-3, -1) : False,
    #(-3, 0) : False,
    #(-3, 1) : False,
    #(-3, 2) : False,
    (-3, 3) : False,
    #(-3, 4) : False,
    (-3, 5) : False,
    
    (-2, -6) : False,
    (-2, -5) : False,
    (-2, -4) : False,
    #(-2, -3) : False,
    #(-2, -2) : False,
    (-2, -1) : False,
    (-2, 0) : False,
    #(-2, 1) : False,
    (-2, 2) : False,
    (-2, 3) : False,
    #(-2, 4) : False,
    (-2, 5) : False,
    
    #(-1, -6) : False,
    (-1, -5) : False,
    #(-1, -4) : False,
    #(-1, -3) : False,
    (-1, -2) : False,
    (-1, -1) : False,
    (-1, 0) : False,
    (-1, 1) : False,
    (-1, 2) : False,
    (-1, 3) : False,
    (-1, 4) : False,
    (-1, 5) : False,
    
    (0, -6) : False,
    (0, -5) : False,
    (0, -4) : False,
    #(0, -3) : False,
    (0, -2) : False,
    #(0, -1) : False,
    (0, 0) : False,
    #(0, 1) : False,
    #(0, 2) : False,
    #(0, 3) : False,
    #(0, 4) : False,
    (0, 5) : False,
    
    (1, -6) : False,
    #(1, -5) : False,
    (1, -4) : False,
    #(1, -3) : False,
    (1, -2) : False,
    #(1, -1) : False,
    (1, 0) : False,
    #(1, 1) : False,
    #(1, 2) : False,
    #(1, 3) : False,
    #(1, 4) : False,
    (1, 5) : False,
    
    (2, -6) : False,
    #(2, -5) : False,
    (2, -4) : False,
    #(2, -3) : False,
    (2, -2) : False,
    #(2, -1) : False,
    (2, 0) : False,
    (2, 1) : False,
    (2, 2) : False,
    (2, 3) : False,
    (2, 4) : False,
    (2, 5) : False,
    
    (3, -6) : False,
    #(3, -5) : False,
    (3, -4) : False,
    (3, -3) : False,
    (3, -2) : False,
    (3, -1) : False,
    (3, 0) : False,
    #(3, 1) : False,
    #(3, 2) : False,
    (3, 3) : False,
    (3, 4) : False,
    #(3, 5) : False,
    
    (4, -6) : False,
    #(4, -5) : False,
    #(4, -4) : False,
    #(4, -3) : False,
    (4, -2) : False,
    (4, -1) : False,
    #(4, 0) : False,
    #(4, 1) : False,
    #(4, 2) : False,
    #(4, 3) : False,
    (4, 4) : False,
    #(4, 5) : False,
    
    (5, -6) : False,
    (5, -5) : False,
    (5, -4) : False,
    #(5, -3) : False,
    (5, -2) : False,
    (5, -1) : False,
    #(5, 0) : False,
    #(5, 1) : False,
    #(5, 2) : False,
    #(5, 3) : False,
    (5, 4) : False,
    (5, 5) : False,
    
    (6, -6) : False,
    (6, -5) : False,
    (6, -4) : False,
    (6, -3) : False,
    (6, -2) : False,
    (6, -1) : False,
    (6, 0) : False,
    (6, 1) : False,
    (6, 2) : False,
    (6, 3) : False,
    (6, 4) : False,
    (6, 5) : False,
}

travelDirections = [(0, 1), (1, 0), (0, -1), (-1, 0)]
movementHistory = []       #History of moves
cellInfos = {}      #point-info dictionary

def clamp(value, minVal, maxVal):
    if (value < minVal):
        return minVal
    elif (value > maxVal):
        return maxVal
    else:
        return value
    
def degTan(degrees):
    return math.tan(math.radians(degrees))

scan = True
goingForward = False
distanceTraveled = 0
oldDistTraveled = 0

World_Up = 90
World_Right = 0
World_Down = 270
World_Left = 180

dir_up = 0
dir_right = 1
dir_down = 2
dir_left = 3
availableDirections = [False, False, False, False]      #up, right, down, left
MeasurementError = 0.2

estimatedPos = point(-5, 5)
    
def SaveData():
    line = ""
    for j in range(5, -6 - 1, -1):      #y
        for i in range(-5, 6 + 1):      #x
            if ((i, j) not in cellInfos):
                line += str(i) + " "
                line += str(j) + " "
                line += str(True) + " "
                line += str(True) + " "
                line += str(True) + " "
                line += str(True) + " "
                line += "\n"
            else:
                line += str(i) + " "
                line += str(j) + " "
                line += str(cellInfos[i, j].t) + " "
                line += str(cellInfos[i, j].r) + " "
                line += str(cellInfos[i, j].u) + " "
                line += str(cellInfos[i, j].l) + " "
                line += "\n"

    f = open(MAP_WRITEPATH, "w")
    f.write(line)
    f.close()
    
def CellReached():      #return False to stop the robot
    estX = int(estimatedPos.x)
    estY = int(estimatedPos.y)
    if ((estX, estY) in visitedCells):
        if (not visitedCells[(estX, estY)]):
            visitedCells[(estX, estY)] = True

    done = True
    for i in range(-5, 6 + 1):
        breakTrigger = False
        for j in range(-6, 5 + 1):
            if ((i, j) in visitedCells):
                if (not visitedCells[(i, j)]):
                    done = False
                    breakTrigger = True
                    break
        if (breakTrigger):
            break
            
    print(estimatedPos)
    if (done):
        robot.stop()
        availableDirections[dir_up] = robot.getWorldAngleLidarRange(World_Up) >= 0.5 + MeasurementError
        availableDirections[dir_right] = robot.getWorldAngleLidarRange(World_Right) >= 0.5 + MeasurementError
        availableDirections[dir_down] = robot.getWorldAngleLidarRange(World_Down) >= 0.5 + MeasurementError
        availableDirections[dir_left] = robot.getWorldAngleLidarRange(World_Left) >= 0.5 + MeasurementError
        
        cellInfos[estimatedPos.x, estimatedPos.y] = cellInfo(estimatedPos.x, estimatedPos.y, availableDirections[dir_up], availableDirections[dir_left], availableDirections[dir_down], availableDirections[dir_right])
        SaveData()
        return False
    
while robot.experiment_supervisor.step(robot.timestep) != -1:
    if (scan):
        availableDirections[dir_up] = robot.getWorldAngleLidarRange(World_Up) >= 0.5 + MeasurementError
        availableDirections[dir_right] = robot.getWorldAngleLidarRange(World_Right) >= 0.5 + MeasurementError
        availableDirections[dir_down] = robot.getWorldAngleLidarRange(World_Down) >= 0.5 + MeasurementError
        availableDirections[dir_left] = robot.getWorldAngleLidarRange(World_Left) >= 0.5 + MeasurementError
        
        #print("North:", availableDirections[dir_up])
        #print("East:", availableDirections[dir_right])
        #print("South:", availableDirections[dir_down])
        #print("West:", availableDirections[dir_left])
        #print(" ")
        
        cellInfos[estimatedPos.x, estimatedPos.y] = cellInfo(estimatedPos.x, estimatedPos.y, availableDirections[dir_up], availableDirections[dir_left], availableDirections[dir_down], availableDirections[dir_right])
        scan = False
    
    if (goingForward == False):
        angle = 360 + 180
        
        searchFail = True
        for i in range(4):
            angle -= 90
            newEstPosX = int(estimatedPos.x + travelDirections[i][0])
            newEstPosX = clamp(newEstPosX, -5, 6)
            newEstPosY = int(estimatedPos.y + travelDirections[i][1])
            newEstPosY = clamp(newEstPosY, -6, 5)
            if (availableDirections[i] and visitedCells[newEstPosX, newEstPosY] == False):
                robot.faceDirProcessError(angle, 2)
                goingForward = True
                estimatedPos = point(newEstPosX, newEstPosY)
                movementHistory.append((travelDirections[i][0], travelDirections[i][1]))
                searchFail = False
                break
        if (searchFail):
            lastMovementInfo = movementHistory.pop(len(movementHistory) - 1)
            newEstPosX = int(estimatedPos.x - lastMovementInfo[0])
            newEstPosX = clamp(newEstPosX, -5, 6)
            newEstPosY = int(estimatedPos.y - lastMovementInfo[1])
            newEstPosY = clamp(newEstPosY, -6, 5)
            targetAngle = 180
            if (lastMovementInfo[0] == -1):     #calculate the opposing angle
                targetAngle = 0
            elif (lastMovementInfo[1] == 1):
                targetAngle = 270
            elif (lastMovementInfo[1] == -1):
                targetAngle = 90
                
            robot.faceDirProcess(targetAngle)
            goingForward = True
            estimatedPos = point(newEstPosX, newEstPosY)
            
    else:
        distanceTraveled =  robot.distanceTraveled() - oldDistTraveled       #distance tracking
        travelSpeed = tilePID.saturatedControlResponse(TRAVEL_SPEED, distanceTraveled, -robot.CONST_MAX_TRAVELSPEED, robot.CONST_MAX_TRAVELSPEED)
        robot.set_left_motors_velocity(travelSpeed)
        robot.set_right_motors_velocity(travelSpeed)
        if (distanceTraveled > 1 - TILE_TRAVEL_ERROR * 2):
            goingForward = False
            scan = True
            oldDistTraveled = robot.distanceTraveled()
            value = CellReached()
            if (value == False):
                break    