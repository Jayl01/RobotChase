# Changes Working Directory to be at the root of FAIRIS-Lite
from collections import namedtuple
import math
import os
os.chdir("../..")

# Import MyRobot Class
from WebotsSim.libraries.MyRobot import MyRobot
from WebotsSim.libraries.MyRobot import entityInfo
from WebotsSim.libraries.MyRobot import goalBotInfos
from WebotsSim.libraries.MyRobot import chaseBotInfos
from WebotsSim.libraries.MyRobot import PIDControl

#Creates path to trackers based on current position
#Follows paths consistently and recreates path every certain amount of tiles

#The data bridge will be dictionary of coordinates.
#Upon moving and entering a new cell, Chase Bot will update the cell that corresponds to its position in the dictionary with "C" (for chase) and the previous cell with the default value
#This dict will be read with coordinates and will represnt a map structure

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
TILE_TRAVEL_ERROR = 0.001
TRAVEL_SPEED = 16
MAP_PATH = "C:/Users/AnotherGuy/FAIRIS-Lite/WebotsSim/controllers/REU_MazeData.txt"
BRIDGE_PATH = "C:/Users/AnotherGuy/FAIRIS-Lite/WebotsSim/controllers/REU_DataBridge.txt"

tilePID = PIDControl(TILE_TRAVEL_ERROR, 1, 1.25)

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
targetCoodinates = []

estimatedPos = point(0, 5)
targetPosition = point(-5, -6)

def ReadData():
    f = open(MAP_PATH, "r")
    for j in range(5, -6 - 1, -1):      #y
        for i in range(-5, 6 + 1):      #x
            data = f.readline().split(' ')
            cellInfos[i, j] = cellInfo(int(data[0]), int(data[1]), data[2] == "True",  data[5] == "True",  data[4] == "True",  data[3] == "True")

    f.close()
    
def WriteBridgedData():
    f = open(BRIDGE_PATH, "r")      #read
    lineResults = f.readlines()
    lineIndex = 0
    amountOfLines = len(lineResults)
    for i in range(amountOfLines):
        lineSplit = lineResults[i].split(',')
        x = int(lineSplit[0])
        y = int(lineSplit[1])
        robotType = lineSplit[2]
        
        if (x == estimatedPos.x and y == estimatedPos.y):
            lineIndex = i
            break;
    
    if (lineIndex < 0 or lineIndex > amountOfLines):
        print("Error writing bride data.")
        return

    writeLine = estimatedPos.x, ",", estimatedPos.y, ", C" 
    f.close()
    f = open(BRIDGE_PATH, "w")
    for i in range(amountOfLines):
        if (i == lineIndex):
            f.write(writeLine)
        else:
            f.write(lineResults[i])

    f.close()
    
def ReadBridgedData():
    f = open(BRIDGE_PATH, "r")      #read
    lineResults = f.readlines()
    for i in range(len(lineResults)):
        lineSplit = lineResults[i].split(',')
        x = int(lineSplit[0])
        y = int(lineSplit[1])
        robotType = lineSplit[2]
        
        if (robotType == "G"):
            targetCoodinates.append(point(x, y))
    
    f.close()

def FindPathToGoal(startX, startY, goalX, goalY):
    pathCreated = False
    cellsToScan = []     #List of points
    cellsScanned = {        #dictionary of points scanned; point-bool
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
        (-4, -5) : False,
        (-4, -4) : False,
        (-4, -3) : False,
        (-4, -2) : False,
        (-4, -1) : False,
        (-4, 0) : False,
        (-4, 1) : False,
        (-4, 2) : False,
        (-4, 3) : False,
        (-4, 4) : False,
        (-4, 5) : False,
    
        (-3, -6) : False,
        (-3, -5) : False,
        (-3, -4) : False,
        (-3, -3) : False,
        (-3, -2) : False,
        (-3, -1) : False,
        (-3, 0) : False,
        (-3, 1) : False,
        (-3, 2) : False,
        (-3, 3) : False,
        (-3, 4) : False,
        (-3, 5) : False,
    
        (-2, -6) : False,
        (-2, -5) : False,
        (-2, -4) : False,
        (-2, -3) : False,
        (-2, -2) : False,
        (-2, -1) : False,
        (-2, 0) : False,
        (-2, 1) : False,
        (-2, 2) : False,
        (-2, 3) : False,
        (-2, 4) : False,
        (-2, 5) : False,
    
        (-1, -6) : False,
        (-1, -5) : False,
        (-1, -4) : False,
        (-1, -3) : False,
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
        (0, -3) : False,
        (0, -2) : False,
        (0, -1) : False,
        (0, 0) : False,
        (0, 1) : False,
        (0, 2) : False,
        (0, 3) : False,
        (0, 4) : False,
        (0, 5) : False,
    
        (1, -6) : False,
        (1, -5) : False,
        (1, -4) : False,
        (1, -3) : False,
        (1, -2) : False,
        (1, -1) : False,
        (1, 0) : False,
        (1, 1) : False,
        (1, 2) : False,
        (1, 3) : False,
        (1, 4) : False,
        (1, 5) : False,
    
        (2, -6) : False,
        (2, -5) : False,
        (2, -4) : False,
        (2, -3) : False,
        (2, -2) : False,
        (2, -1) : False,
        (2, 0) : False,
        (2, 1) : False,
        (2, 2) : False,
        (2, 3) : False,
        (2, 4) : False,
        (2, 5) : False,
    
        (3, -6) : False,
        (3, -5) : False,
        (3, -4) : False,
        (3, -3) : False,
        (3, -2) : False,
        (3, -1) : False,
        (3, 0) : False,
        (3, 1) : False,
        (3, 2) : False,
        (3, 3) : False,
        (3, 4) : False,
        (3, 5) : False,
    
        (4, -6) : False,
        (4, -5) : False,
        (4, -4) : False,
        (4, -3) : False,
        (4, -2) : False,
        (4, -1) : False,
        (4, 0) : False,
        (4, 1) : False,
        (4, 2) : False,
        (4, 3) : False,
        (4, 4) : False,
        (4, 5) : False,
    
        (5, -6) : False,
        (5, -5) : False,
        (5, -4) : False,
        (5, -3) : False,
        (5, -2) : False,
        (5, -1) : False,
        (5, 0) : False,
        (5, 1) : False,
        (5, 2) : False,
        (5, 3) : False,
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
    resultList = { (startX, startY) : (0, 0) }        #dictionary of point traces; point-point (pos-dir)

    targetPosition = point(goalX, goalY)
    newCellsToScan = [targetPosition]
    tries = 0
    while (pathCreated == False):
        cellsToScan = newCellsToScan.copy()
        newCellsToScan.clear()
        #print(cellsToScan, len(cellsToScan))
        
        tries += 1
        if (tries >= 500):
            break

        for i in range(len(cellsToScan)):
            cellToScan = cellsToScan[i]
            cellsScanned[cellToScan] = True
            
            cellAbove = cellInfos[cellToScan[0], cellToScan[1] + 1 if cellToScan[1] + 1 <= 5 else 5]     #y = a +y clamp
            cellUnder = cellInfos[cellToScan[0], cellToScan[1] - 1 if cellToScan[1] - 1 >= -6 else -6]      #y = a -y clamp
            cellLeft = cellInfos[cellToScan[0] - 1 if cellToScan[0] - 1 >= -5 else -5, cellToScan[1]]      #x = a -x clamp
            cellRight = cellInfos[cellToScan[0] + 1 if cellToScan[0] + 1 <= 6 else 6, cellToScan[1]]        #x = a +x clamp
            
            cellAbovePoint = (cellAbove.x, cellAbove.y)
            if (cellInfos[cellToScan].t and cellsScanned[cellAbovePoint] == False):
                cellsScanned[cellAbovePoint] = True
                newCellsToScan.append(cellAbovePoint)
                resultList[cellAbovePoint] = (0, 1)
            
            cellLeftPoint = (cellLeft.x, cellLeft.y)
            if (cellInfos[cellToScan].l and cellsScanned[cellLeftPoint] == False):
                cellsScanned[cellLeftPoint] = True
                newCellsToScan.append(cellLeftPoint)
                resultList[cellLeftPoint] = (-1, 0)
            
            cellRightPoint = (cellRight.x, cellRight.y)
            if (cellInfos[cellToScan].r and cellsScanned[cellRightPoint] == False):
                cellsScanned[cellRightPoint] = True
                newCellsToScan.append(cellRightPoint)
                resultList[cellRightPoint] = (1, 0)
            
            cellUnderPoint = (cellUnder.x, cellUnder.y)
            if (cellInfos[cellToScan].u and cellsScanned[cellUnderPoint] == False):
                cellsScanned[cellUnderPoint] = True
                newCellsToScan.append(cellUnderPoint)
                resultList[cellUnderPoint] = (0, -1)
                
            if (cellToScan == (estimatedPos.x, estimatedPos.y)):      #Robot location
                pathCreated = True
                break
            
        #print(newCellsToScan)
        cellsToScan.clear()
        
    return resultList      #basically you loop this from your current cell to find a path to the goal
    #for j in range(3, -1, -1):
    #    for i in range(0, 3 + 1):
    #        if ()
    
def CellReached():      #return False to stop the robot
    estX = int(estimatedPos.x)
    estY = int(estimatedPos.y)
    print(" ")
    print(" ")
    print("Chase Bot")
    print("Current Pos:", estimatedPos)
    chaseBotInfos[0] = entityInfo(estimatedPos.x, estimatedPos.y, 1)
    print(len(goalBotInfos))
    pathToFollow = FindPathToGoal(estimatedPos.x, estimatedPos.y, goalBotInfos[0].x, goalBotInfos[0].y)
    #pathToFollow = FindPathToGoal(estX, estY)           #recalculate
    done = estimatedPos == targetPosition
            
    if (done):
        robot.stop()
        return False
    

ReadData()
print(len(goalBotInfos))
chaseBotInfos.append(entityInfo(estimatedPos.x, estimatedPos.y, 1))
pathToFollow = FindPathToGoal(estimatedPos.x, estimatedPos.y, -5, -6)

operationTerminated = True

while robot.experiment_supervisor.step(robot.timestep) != -1:
    
    if (goingForward == False):
        if ((estimatedPos.x, estimatedPos.y) == targetPosition):
            robot.stop()
            print("GOAL")
            break
        
        #print("Value:", pathToFollow[int(estimatedPos.x), int(estimatedPos.y)])
        nextMovementInfo = pathToFollow.pop((int(estimatedPos.x), int(estimatedPos.y)))
        newEstPosX = int(estimatedPos.x - nextMovementInfo[0])
        newEstPosX = clamp(newEstPosX, -5, 6)
        newEstPosY = int(estimatedPos.y - nextMovementInfo[1])
        newEstPosY = clamp(newEstPosY, -6, 5)
        targetAngle = 180
        if (nextMovementInfo[0] == -1):     #calculate the opposing angle
            targetAngle = 0
        elif (nextMovementInfo[1] == 1):
            targetAngle = 270
        elif (nextMovementInfo[1] == -1):
            targetAngle = 90
                
        robot.faceDirProcess(targetAngle)
        goingForward = True
        estimatedPos = point(newEstPosX, newEstPosY)
        if (estimatedPos.x != newEstPosX or estimatedPos.y != newEstPosY):
            CellReached()
            
    else:
        distanceTraveled =  robot.distanceTraveled() - oldDistTraveled       #distance tracking
        travelSpeed = tilePID.saturatedControlResponse(robot.CONST_TRAVEL_SPEED, distanceTraveled, -robot.CONST_MAX_TRAVELSPEED, robot.CONST_MAX_TRAVELSPEED)
        robot.set_left_motors_velocity(travelSpeed)
        robot.set_right_motors_velocity(travelSpeed)
        #print(distanceTraveled)
        if (distanceTraveled > 1 - TILE_TRAVEL_ERROR * 2):
            goingForward = False
            scan = True
            oldDistTraveled = robot.distanceTraveled()
            value = CellReached()
            if (value == False):
                break    