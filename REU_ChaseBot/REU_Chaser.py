# Changes Working Directory to be at the root of FAIRIS-Lite
from collections import namedtuple
import math
import os
os.chdir("../..")

# Import MyRobot Class
from WebotsSim.libraries.MyRobot import MyRobot
from WebotsSim.libraries.MyRobot import PIDControl

#Creates path to trackers based on current position
#Follows paths consistently and recreates path every certain amount of tiles

# Create the robot instance.
robot = MyRobot()
cellInfo = namedtuple('tileInfo', 'x y t l u r')        #top left under right

# Move robot to a random staring position listed in maze file
robot.move_to_start()

LIDAR_FRONT = 400
LIDAR_RIGHT = 600
LIDAR_BACK = 0     #or 799
LIDAR_LEFT = 200

FRONT_LIMIT = 0.1
SIDE_LIMIT = 0.4
TILE_TRAVEL_ERROR = 0.001
TRAVEL_SPEED = 20
MAP_READPATH = "C:/Users/AnotherGuy/FAIRIS-Lite/WebotsSim/controllers/REU_MazeData.txt"

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

estimatedPos = robot.starting_position
estimatedPos.x += 2
estimatedPos.y += 2

def PrintMap():
    
    availableDirections[dir_up] = robot.getWorldAngleLidarRange(World_Up) >= 0.5 + MeasurementError
    availableDirections[dir_right] = robot.getWorldAngleLidarRange(World_Right) >= 0.5 + MeasurementError
    availableDirections[dir_down] = robot.getWorldAngleLidarRange(World_Down) >= 0.5 + MeasurementError
    availableDirections[dir_left] = robot.getWorldAngleLidarRange(World_Left) >= 0.5 + MeasurementError
        
    cellInfos[estimatedPos.x, estimatedPos.y] = cellInfo(estimatedPos.x, estimatedPos.y, availableDirections[dir_up], availableDirections[dir_left], availableDirections[dir_down], availableDirections[dir_right])
    
def SaveData():
    line = ""
    for j in range(3, -1, -1):
        for i in range(0, 3 + 1):
            line += str(i) + " "
            line += str(j) + " "
            line += str(cellInfos[i, j].t) + " "
            line += str(cellInfos[i, j].r) + " "
            line += str(cellInfos[i, j].u) + " "
            line += str(cellInfos[i, j].l) + " "
            line += "\n"

    f = open("C:/Users/AnotherGuy/FAIRIS-Lite/WebotsSim/controllers/Lab5_Task1_Controller/maze2Data.txt", "w")
    f.write(line)
    f.close()
    
def CellReached():      #return False to stop the robot
    estX = int(estimatedPos.x)
    estY = int(estimatedPos.y)

    done = True
            
    if (done):
        robot.stop()
        PrintMap()
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
            newEstPosX = clamp(newEstPosX, 0, 3)
            newEstPosY = int(estimatedPos.y + travelDirections[i][1])
            newEstPosY = clamp(newEstPosY, 0, 3)
            if (availableDirections[i]):
                robot.faceDirProcess(angle)
                goingForward = True
                estimatedPos.x = newEstPosX
                estimatedPos.y = newEstPosY
                movementHistory.append((travelDirections[i][0], travelDirections[i][1]))
                searchFail = False
                break
        if (searchFail):
            lastMovementInfo = movementHistory.pop(len(movementHistory) - 1)
            newEstPosX = int(estimatedPos.x - lastMovementInfo[0])
            newEstPosX = clamp(newEstPosX, 0, 3)
            newEstPosY = int(estimatedPos.y - lastMovementInfo[1])
            newEstPosY = clamp(newEstPosY, 0, 3)
            targetAngle = 180
            if (lastMovementInfo[0] == -1):     #calculate the opposing angle
                angle = 0
            elif (lastMovementInfo[1] == 1):
                angle = 270
            elif (lastMovementInfo[1] == -1):
                angle = 90
                
            robot.faceDirProcess(angle)
            goingForward = True
            estimatedPos.x = newEstPosX
            estimatedPos.y = newEstPosY
            
    else:
        distanceTraveled =  robot.distanceTraveled() - oldDistTraveled       #distance tracking
        travelSpeed = tilePID.saturatedControlResponse(robot.TRAVEL_SPEED, distanceTraveled, -robot.CONST_MAX_TRAVELSPEED, robot.CONST_MAX_TRAVELSPEED)
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