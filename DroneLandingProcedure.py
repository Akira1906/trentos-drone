# Python client example to get Lidar data from a drone, although this script works for any AirSim-supported vehicle
# This script is for Lidar sensors using 'SensorLocalFrame' as DataFrame under settings.json.
# Sample settings.json used for this script:
'''
{
    "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings_json.md",
    "SettingsVersion": 1.2,
  
    "SimMode": "Multirotor",
  
     "Vehicles": {
        "Drone1": {
            "VehicleType": "SimpleFlight",
            "AutoCreate": true,
            "Sensors": {
                "Distance": {
                    "SensorType": 5,
                    "Enabled" : true,
                    "Yaw": 0, "Pitch": -90, "Roll": 0
                },
                "LidarSensorHor": { 
                    "SensorType": 6,
                    "Enabled" : true,
                    "NumberOfChannels": 1,
                    "RotationsPerSecond": 10,
                    "Range":100,
                    "PointsPerSecond": 2000,
                    "X": 0, "Y": 0, "Z": -1,
                    "Roll": 0, "Pitch": 0, "Yaw" : 0,
                    "VerticalFOVUpper": 0,
                    "VerticalFOVLower": 0,
                    "HorizontalFOVStart": 0,
                    "HorizontalFOVEnd": 0,
                    "DrawDebugPoints": true,
                    "DataFrame": "SensorLocalFrame"
                },
                "LidarSensorVer1": { 
                  "SensorType": 6,
                  "Enabled" : true,
                  "NumberOfChannels": 1,
                  "RotationsPerSecond": 10,
                  "Range":100,
                  "PointsPerSecond": 1000,
                  "X": 0, "Y": 0, "Z": -1,
                  "Roll": 0, "Pitch": 90, "Yaw" : 0,
                  "VerticalFOVUpper": 0,
                  "VerticalFOVLower": 0,
                  "HorizontalFOVStart": 0,
                  "HorizontalFOVEnd": 0,
                  "DrawDebugPoints": true,
                  "DataFrame": "SensorLocalFrame"
              },
              "LidarSensorVer2": { 
                "SensorType": 6,
                "Enabled" : true,
                "NumberOfChannels": 1,
                "RotationsPerSecond": 10,
                "Range":100,
                "PointsPerSecond": 1000,
                "X": 0, "Y": 0, "Z": -1,
                "Roll": 0, "Pitch": 90, "Yaw" : 90,
                "VerticalFOVUpper": 0,
                "VerticalFOVLower": 0,
                "HorizontalFOVStart": 0,
                "HorizontalFOVEnd": 0,
                "DrawDebugPoints": true,
                "DataFrame": "SensorLocalFrame"
            }
            }
        }
    }
  }
'''

import airsim
import numpy as np
import pprint
import time

class LidarTest:

    def __init__(self):
         #connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        #self.client.reset()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        print('Connected!\n')

#---------------------------------------------------------------------------------------
# DEBUGGING FUNCTIONS
#---------------------------------------------------------------------------------------

    def test(self, vehicleName, distanceSensorName):
        self.client.takeoffAsync()
        self.client.moveByRollPitchYawZAsync(roll=0, pitch=0.125*np.pi, yaw=np.pi,z=-5, duration=5, vehicle_name=vehicleName)
        distanceData = self.client.getDistanceSensorData(distanceSensorName, vehicleName)
        print(distanceData)

    def print_state(self):
        print("===============================================================")
        state = self.client.getMultirotorState()
        print("state: %s" % pprint.pformat(state))
        return state
    
    def debugShowPointPosition(self, coordinates):
        coordinatesVector = [airsim.Vector3r(coordinates[0], coordinates[1], coordinates[2]), airsim.Vector3r(coordinates[0], coordinates[1], coordinates[2] - 20)]
        self.client.simPlotLineStrip(coordinatesVector, color_rgba=[0.0, 1.0, 0.0, 1.0], thickness=30.0, duration=60.0, is_persistent=False)

#---------------------------------------------------------------------------------------
# GENERAL DATA PARSING/PROCESSING
#---------------------------------------------------------------------------------------

    """ 
        Takes a pointcloud which contains only similar z values.
        1. Determine the number of objects dislayed by the pointcloud and which points belong to them.
        2. Determines the average point in space of an object detected earlier
        3. Determines the distance of the objects to the drone
        4. Chooses the closest object to the drone and returns its average point
        Args:
            self
            lidarName (str)             : Name of the lidar we got the pointcloud data from
            filteredPointcloud([[str]]) : Pre-filtered list of points as Strings
            vehicleName (str)           : Name of the vehicle controlled

        Returns:
            closestObjectMiddlePoint ([float, float, float]) : Coordinates of the middlepoint of an detected object closest to the drone

    """ 
    def getClosestObjectMiddlePoint(self, lidarName, filteredPointcloud, vehicleName):
        middlePoints = self.getObjectPositionsInPointcloud(filteredPointcloud)

        #1. determine distance to the drone and therefore choose the closest landing point if there is more than 1
        if len(middlePoints) > 1:
            
            lidar_data = self.client.getLidarData(lidar_name=lidarName, vehicle_name=vehicleName)
            position = lidar_data.pose.position
            distances = []
            for mPoint in middlePoints:
                distances.append(
                    np.sqrt(
                        pow(mPoint[0] - position.x_val, 2) + pow(mPoint[1] - position.y_val, 2)
                    )
                )
            #2. choose the middlePoint with the lowest distance to the drone
            closestObjectMiddlePoint = middlePoints[distances.index(min(distances))]
        else:
            closestObjectMiddlePoint = middlePoints[0]
        
        return closestObjectMiddlePoint


    """ 
        Takes a pointcloud which contains preferrably only one rotation of the lidar, so objects won't be detected twice.
        1. Determine the number of objects dislayed by the pointcloud and which points belong to them.
        2. Determines the average point in space of an object detected earlier
        Args:
            self
            pointcloud      ([[float, float, float]])       : List of a coordinate list

        Returns:
            middlePoints    ([float, float, float]) : estimated middlepoints of the objects scanned by the lidar 

    """ 
    def getObjectPositionsInPointcloud(self, pointcloud):
        #1. detect if the points belong to the same object, if the distance between them exceeds the maxdistance

        #TODO determine the correct value for the lidar settings
        maxDistance = 12 #maximum distance between two points so they belong to the same object

        tempLastPoint = pointcloud[0]
        objects = [[tempLastPoint]] #list of detected objects that contains the pointclouds which belong to an object

        for point in pointcloud[1:]:
            #determine distance between two neighbouring points
            x = point[0] - tempLastPoint[0]
            y = point[1] - tempLastPoint[1]
            tempLastPoint = point

            distance = np.sqrt(pow(x,2) + pow(y,2))

            if(distance > maxDistance):
                objects.append([point])#add new objects to list
            else:
                objects[-1].append(point)#add new point to an objects          

        #2. determines the two points with the greatest distance between each other
        # (first and last point of a pointcloud belonging to an object) and averages them

        middlePoints = []
        for pointcloud in objects:
            middlePoints.append(
                [(pointcloud[0][0] + pointcloud[-1][0])/2,
                (pointcloud[0][1] + pointcloud[-1][1])/2, pointcloud[0][2]])
                
        return middlePoints


    """ 
        Filters a raw pointCloudList for their an exact z value only and casts to float.
        Args:
            self
            pointcloudList ([[str]])    : List of Points that we want to filter on
            zFilter         (str)       : Z value on that we want to filter on, should be a float as a string

        Returns:
            pointcloudList ([[float, float, float]])    : Filtered list

    """ 
    def exactFilterPoints(self, pointcloudList, zFilter):
        filteredPoints = []

        #filter for zFilter exactly
        for point in pointcloudList:
            point = point.split()
            if(point[2] == zFilter):               
                filteredPoints.append([float(point[0]), float(point[1]), float(point[2])]) #get rid of the color data, only parse coordinates

        return filteredPoints


    """ 
        Filters a raw pointCloudList for their roughly lowest z value with a defined accuracy and casts to float
        Args:
            self
            pointcloudList ([[str]]) : List of Points that we want to filter on
            accuracy       (float)   : Accuracy of the filtering

        Returns:
            pointcloudList ([[float, float, float]])    : Filtered list
            lowestZ (float)                             : Lowest Z value detected

    """ 
    def roughFilterHighestPoints(self, pointcloudList, accuracy):
        lowestZ = 0
        #determine lowest Z value
        for point in pointcloudList:
            point = point.split()
            if float(point[2]) < lowestZ:
                lowestZ = float(point[2])
        filteredPoints = []

        #filters for lowestZ with accuracy
        for point in reversed(pointcloudList):
            point = point.split()
            point = [float(point[0]), float(point[1]), float(point[2])]
            if(point[2] < lowestZ + accuracy and point[2] > lowestZ - accuracy):
                filteredPoints.append(point)

        return filteredPoints, lowestZ
#---------------------------------------------------------------------------------------
# LIDAR SCAN Functions
#---------------------------------------------------------------------------------------    


    """ 
        Scans the environment using lidar sensors.
        Converts the lidar data into pointclouds and writes them to a file.
        Depending on the flightSequence the function exits with different criteria.
        Args:
            self
            vehicleName     (str)                   : Vehicle Name
            lidarNames      ([str])                 : List of the lidar names
            distanceName    (str)                   : Distance sensor name
            landingPosition ([float, float, float]) : List of the 3 coordinates of the landing point

    """
    def executeScan(self, vehicle_name, lidarNames, flightSequence):
        print('Scanning Has Started\n')
        existing_data_cleared = False   #change to true to superimpose new scans onto existing .asc files
        lidarDetected = {} #Dictionary{lidar_name : 0 (nothing detected)/ 1 (something detected)}
        for lidar_name in lidarNames:
            lidarDetected.update({lidar_name : 1})

 
        try:
            while flightSequence == 0 or flightSequence == 2:
                #------------------------------------------------
                # Write LidarSensorData to file
                #------------------------------------------------
                for lidar_name in lidarNames:
                    filename = f"{vehicle_name}_{lidar_name}_pointcloud.asc"
                    if not existing_data_cleared:
                        f = open(filename,'w')
                    else:
                        f = open(filename,'a')
                    lidar_data = self.client.getLidarData(lidar_name=lidar_name, vehicle_name=vehicle_name)

                    #------------------------------------------------
                    # Magic Pointcloud calculations
                    #------------------------------------------------

                    orientation = lidar_data.pose.orientation
                    q0, q1, q2, q3 = orientation.w_val, orientation.x_val, orientation.y_val, orientation.z_val
                    rotation_matrix = np.array(([1-2*(q2*q2+q3*q3),2*(q1*q2-q3*q0),2*(q1*q3+q2*q0)],
                                                    [2*(q1*q2+q3*q0),1-2*(q1*q1+q3*q3),2*(q2*q3-q1*q0)],
                                                    [2*(q1*q3-q2*q0),2*(q2*q3+q1*q0),1-2*(q1*q1+q2*q2)]))

                    position = lidar_data.pose.position

                    for i in range(0, len(lidar_data.point_cloud), 3):
                        xyz = lidar_data.point_cloud[i:i+3]

                        corrected_x, corrected_y, corrected_z = np.matmul(rotation_matrix, np.asarray(xyz))
                        final_x = corrected_x + position.x_val
                        final_y = corrected_y + position.y_val
                        final_z = corrected_z + position.z_val

                        f.write("%f %f %f %d %d %d \n" % (final_x,final_y,final_z,255,255,0))
                    f.close()

                    #------------------------------------------------
                    # Note down if the lidar detected something or not
                    #------------------------------------------------
                    if(lidar_data.point_cloud == []):
                        lidarDetected[lidar_name] = 0
                    else:
                        lidarDetected[lidar_name] = 1

                existing_data_cleared = True

            
                #------------------------------------------------
                # Exit program according to the flightSequence and detected objects
                #------------------------------------------------

                #flightSequence 0

                if(flightSequence == 0 and lidarDetected["LidarSensorHor"] == 0 ):
                    print("FlightSequence 0: Scanned the whole environment")
                    return
                elif(flightSequence == 0):
                    self.client.moveByVelocityBodyFrameAsync(0,0,-5,0.5).join()

                #flightSequence 2

                if(flightSequence == 2 and lidarDetected[lidarNames[0]] == 1 and lidarDetected[lidarNames[1]] == 1):
                    print("FlightSequence2: Scanned for landing point")
                    return
                #elif(flightSequence == 2):
                    #self.client.moveByRollPitchYawThrottleAsync(0,0,40,1,0.5, "drone1")


            

        except KeyboardInterrupt:
            print("Quitted with Keyboard Interrupt!\n")


#---------------------------------------------------------------------------------------
# FLIGHT PROCEDURE CONTROL Functions
#---------------------------------------------------------------------------------------  

#Flight Sequence 0

    """ 
        Evaluates the pointcloud detected with the horizontal lidar sensor.
        Detects possible landing Targets.
        Chooses the one with the closest proximity to the drone.
        Args:
            self
            filename    (str)       : Name of thepointcloud file
            lidarName   (str)       : Lidar name of the horizontasl lidar used
            vehicleName (str)       : Name of the vehicle controlled

        Returns:
            landingTarget ([float, float, float]) : Rough position of the chosen landing platform
    """ 
    def evaluateLandingTarget(self, filename, lidarName, vehicleName):

        #reads the lidar pointcloud file450
        print("evaluateLandingTarget()")
        f = open(filename,'r')
        rawLines = f.readlines()
        f.close()

        if(len(rawLines) < 2):
            raise Exception("evaluateLandingTarget.Error: LidarData File empty/too short")

        #Filters a pointCloudList for their lowest z value only and casts to float
        #get lowestZ value which is in the end of the pointcloud file
        lowestZ = rawLines[-1].split()[2]
        lidarPoints = self.exactFilterPoints(rawLines, lowestZ)       
        

        #detect objects and their pointcloud points
        maxdistance = 5 #maximum distance between two points so they belong to the same landingSpot TODO
        tempLastPoint = lidarPoints[0]
        landingSpots = [[tempLastPoint]] #list of detected landingSpots that contains the lidarpoints which belong to a landingSpot
        for point in lidarPoints[1:]:
            #determine distance between two neighbouring points
            x = point[0] - tempLastPoint[0]
            y = point[1] - tempLastPoint[1]
            tempLastPoint = point

            distance = np.sqrt(pow(x,2) + pow(y,2))
            distanceToOrigin = np.sqrt(pow(point[0],2) + pow(point[1],2))
            print(distance, distanceToOrigin)
            if(distance > maxdistance):
                landingSpots.append([point])
            else:
                landingSpots[-1].append(point)
        
        print("Detected " + str(len(landingSpots)) + " possible Landing Spots")
        print("choosing a landing spot...")

        #determine the average landingPoint position of a landingSpot
        landingPoints = len(landingSpots) * [[0.0, 0.0, 0.0]]
        for i in range(len(landingSpots)):
            averageX = 0
            averageY = 0
            for point in landingSpots[i]:
                averageX += point[0]
                averageY += point[1]
            averageX /= len(landingSpots[i])
            averageY /= len(landingSpots[i])
            landingPoints[i] = [averageX, averageY, landingSpots[i][0][2]]
        
        #determine the distance of the landing spots to the drone
        lidar_data = self.client.getLidarData(lidar_name=lidarName, vehicle_name=vehicleName)
        position = lidar_data.pose.position
        distances = []
        for mPoint in landingPoints:
            distances.append(
                np.sqrt(
                    pow(mPoint[0] - position.x_val, 2) + pow(mPoint[1] - position.y_val, 2)
                )
            )
            
        #choose the landingPoint with the lowest distance to the drone
        landingTarget = landingPoints[distances.index(max(distances))]

        #NEW
        maxDistance = 12 #TODO maximum distance between two points so they belong to the same object

        objectsCoordinatesPerLevel = []



        #determine object positions for every z-scan level
        while (len(rawLines) > 0):
            objectsCoordinates, levelLen = self.detectObjectsOnLevel(rawLines)
            objectsCoordinatesPerLevel += [objectsCoordinates]
            rawLines = rawLines[levelLen:]

            #for point in objectsCoordinatesPerLevel[-1]:
                #self.debugShowPointPosition(point)

        objectData = objectsCoordinatesPerLevel[-1] # List of Lists containing the average X and Y coordinates and the minimum Z height of an object

        for objectsCoordinates in reversed(objectsCoordinatesPerLevel[:-1]):
            for object in objectsCoordinates:
                found = False
                for oldObject in objectData:
                    x = object[0] - oldObject[0]
                    y = object[1] - oldObject[1]
                    #Performanceoptimierung mittels conditions möglich
                    distance = np.sqrt(pow(x,2) + pow(y,2))
                    if(distance < maxDistance):
                        oldObject[0] = (oldObject[0] + object[0])/2
                        oldObject[1] = (oldObject[1] + object[1])/2
                        found = True
                        break
                if not found:
                    objectData += [object]


        return landingTarget, objectData

    """ 
        Evaluates the pointcloud data received by the horizontal lidar sensor to determine where in the world objects are.
        Objects are only detected, if they are physically connected to the ground, floating objects will not be detected.
        Args:
            self
            pointcloudList    ([[String]])                  : raw Lidardata pointcloud as String
        Returns:
            objectsCoordinates  ([[float, float float]])    : coordinates of the rough coordinates of the objects detected on the ground level by lidar
    """
    def detectObjectsOnLevel(self, pointcloudList):
        #the highestZ value is found in the first rotation of the horizontal lidar, that means in the beginning of the file
        highestZ = pointcloudList[0].split()[2]
        lowestLidarPoints = self.exactFilterPoints(pointcloudList, highestZ)
        
        objectsCoordinates = self.getObjectPositionsInPointcloud(lowestLidarPoints)

        return objectsCoordinates, len(lowestLidarPoints)


#-----------------
#Flight Sequence 1

    """ 
        Steers the drone to the landing position until the landing platform is detected below the
        drone with the distance Sensor
        Args:
            self
            vehicleName     (str)                   : Vehicle Name
            lidarName       (str)                   : Any lidar name
            distanceName    (str)                   : Distance sensor name
            landingPosition ([float, float, float]) : List of the 3 coordinates of the landing point

    """
    def flyToLandingPosition(self, vehicleName, lidarNames, distanceName, landingPosition):
        #determines the position of the landing platform relative to the drone
        currPosition = self.client.getLidarData(lidarNames[0], vehicleName).pose.position
        xDifference = landingPosition[0] - currPosition.x_val
        yDifference = landingPosition[1] - currPosition.y_val
        distanceToDrone = np.sqrt(pow(xDifference, 2) + pow(yDifference, 2))

        #calculate the direction at which the drone has to fly to reach the landingPosition
        yawAngle = - np.arccos(xDifference/distanceToDrone)

        #fly in the direction of the landingPosition until the distance sensor detects the landing platform
        flightPitch = 0.02 * np.pi
        self.client.moveByRollPitchYawZAsync(
                roll = 0, pitch = 0, yaw = yawAngle,
                z = landingPosition[2]-3, duration = 1,
                vehicle_name = vehicleName).join()

        distanceSensorData = self.client.getDistanceSensorData(distanceName, vehicleName)
        while(distanceSensorData.distance > 6):
            self.client.moveByRollPitchYawZAsync(
                roll = 0, pitch = flightPitch, yaw = yawAngle,
                z = landingPosition[2]-3, duration = 0.05,
                vehicle_name = vehicleName)

            distanceSensorData = self.client.getDistanceSensorData(distanceName, vehicleName)
            print(distanceSensorData.distance)
            
        self.client.moveByRollPitchYawZAsync(
            roll = 0, pitch = -6 * flightPitch, yaw = yawAngle,
            z = landingPosition[2]-3, duration = 3,
            vehicle_name = vehicleName).join()


#-----------------
#Flight Sequence 2

    """ 
        Parses the lidar files extracted by the vertical lidars 
        and detects the center of the landing spot close to the drone.
        Args:
            self
            filename1   (str)   : Name of the first pointcloud file
            filename2   (str)   : Name of the second pointcloud file
            lidarNames  ([str]) : List of lidar names of the two vertical lidars used
            vehicleName (str)   : Name of the vehicle controlled

        Returns:
            landingTarget ([float, float, float]) : Determined best landing position

    """ 
    def detectExactLandingPoint(self, filename1, filename2, lidarNames, vehicleName):
        print("detectExactLandingPoint()")
        #read in the data from the lidar pointcloud files
        f = open(filename1, 'r')
        rawLines1 = f.readlines()
        f.close()
        f = open(filename2, 'r')
        rawLines2 = f.readlines()
        f.close()

        if(len(rawLines1) == 0 or len(rawLines2) == 0):
            raise Exception("detectExactLandingPoint.Error: LidarData File empty/too short")

        #filter for points on the approximate height of the landing spot and gets the Z position of it
        highestPointcloud1, lowestZ1 = self.roughFilterHighestPoints(rawLines1, 0.1)
        highestPointcloud2, lowestZ2 = self.roughFilterHighestPoints(rawLines2, 0.1)
        lowestZ = min(lowestZ1, lowestZ2)

        #if one of the lidars didn't detected another plattform roughly minimum 1 meter lower ignore it 
        ignoreLidar = 0 # 0 - nothing ignored, 1 - lidar1 ignored, 2 - lidar2 ignored
        if(abs(lowestZ1 - lowestZ2) > 1):
            if(lowestZ == lowestZ1):
                ignoreLidar = 2
            else:
                ignoreLidar = 1

        #determine the possible landingSpots detected on the radar and choose the one closest to the drone
        if(ignoreLidar == 2 or ignoreLidar == 0):
            middlePoint1 = self.getClosestObjectMiddlePoint(lidarNames[0], highestPointcloud1, vehicleName)
            if(len(highestPointcloud1) == 0):
                raise Exception("detectExactLandingPoint.Error: Filtered pointclouds empty")

        if(ignoreLidar == 1 or ignoreLidar == 0):
            middlePoint2 = self.getClosestObjectMiddlePoint(lidarNames[1], highestPointcloud2, vehicleName) 
            if(len(highestPointcloud2) == 0):
                raise Exception("detectExactLandingPoint.Error: Filtered pointclouds empty")

        #averages the two axis' middlepoints of the object to find a safe landing position in the center of the object
        if(ignoreLidar == 1):
            landingTarget = [middlePoint2[0], middlePoint2[1], lowestZ]

        elif(ignoreLidar == 2):
            landingTarget = [middlePoint1[0], middlePoint1[1], lowestZ]

        else:
            landingTarget = [
                (middlePoint1[0] + middlePoint2[0])/2,
                (middlePoint1[1] + middlePoint2[1])/2,
                lowestZ]

        return landingTarget


#-----------------
#Flight Sequence 3

    """ 
        Parses the lidar files extracted by the vertical lidars 
        and detects the center of the landing spot close to the drone.
        Args:
            self
            vehicleName (str)   : Name of the vehicle controlled
            lidarNames  ([str]) : List of lidar names of the two vertical lidars used
            distanceName (str)  : Name of the distance sensor
            landingPositoin ([float, float, float]) : Coordinates of the desired landing position
    """ 
    def land(self, vehicleName, lidarNames, distanceName, landingPosition):
        #determines the position of the landing platform relative to the drone
        currPosition = self.client.getLidarData(lidarNames[0], vehicleName).pose.position
        xDifference = landingPosition[0] - currPosition.x_val
        yDifference = landingPosition[1] - currPosition.y_val
        z = currPosition.z_val
        distanceToDrone = np.sqrt(pow(xDifference, 2) + pow(yDifference, 2))

        #calculate the direction at which the drone has to fly to reach the landingPosition
        #yawAngle = - np.arccos(xDifference/distanceToDrone)

        #fly in the direction of the landingPosition until the distance sensor detects the landing platform

        distanceSensorData = self.client.getDistanceSensorData(distanceName, vehicleName)
        while(distanceSensorData.distance > 4.0 or distanceToDrone > 1.0):

            self.client.moveByVelocityZAsync(xDifference, yDifference, z, 0.2, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 90)).join()
            distanceSensorData = self.client.getDistanceSensorData(distanceName, vehicleName)
            print(distanceSensorData.distance)
            #determine distance to landingPosition
            currPosition = self.client.getLidarData(lidarNames[0], vehicleName).pose.position
            xDifference = landingPosition[0] - currPosition.x_val
            yDifference = landingPosition[1] - currPosition.y_val
            distanceToDrone = np.sqrt(pow(xDifference, 2) + pow(yDifference, 2))

        self.client.moveByVelocityBodyFrameAsync(0,0,2.5,2).join()
        print("landing success")
        

# main
if __name__ == "__main__":
    lidarTest = LidarTest()
    lidarNames = ["LidarSensorHor", "LidarSensorVer1", "LidarSensorVer2"]
    distanceName = "Distance"
    vehicleName = "Drone1"

    lidarTest.debugShowPointPosition([23, -12, 0])

    #lidarTest.test(vehicleName, distanceName)
    # flightSequence: 0 - detect highest point, 1 - fly to highest position, 2 - detect landing point, 3 - landing
    #-----------------------
    flightSequence = 0 
    print("Sequence 0: detect highest point")
    lidarTest.executeScan('Drone1',lidarNames[0:1], flightSequence)
    landingTarget, objectDataToSave = lidarTest.evaluateLandingTarget(
        vehicleName+"_" + lidarNames[0] + "_pointcloud.asc",
         lidarNames[0],
          vehicleName)
    
    lidarTest.debugShowPointPosition(landingTarget)

    for object in objectDataToSave:
        lidarTest.debugShowPointPosition(object)
        print(object)
    
    #-----------------------
    flightSequence = 1
    print("Sequence 1: fly to highest landing spot")
    print("Sequence 1: landingTarget: "+str(landingTarget))
    #lidarTest.client.moveToPositionAsync(landingTarget[0], landingTarget[1], landingTarget[2]-3, 10).join()
    lidarTest.flyToLandingPosition(vehicleName, lidarNames[0], distanceName, landingTarget)
    print("Sequence 1: reached landing destination")
    #time.sleep(5)
    
    #-----------------------
    flightSequence = 2
    print("Sequence 2: detect landing point")
    lidarTest.executeScan(vehicleName, lidarNames[1:], flightSequence)
    landingTarget1 = lidarTest.detectExactLandingPoint(
        vehicleName + "_" + lidarNames[1] + "_pointcloud.asc",
         vehicleName + "_" + lidarNames[2] + "_pointcloud.asc",
          lidarNames[1:],
           vehicleName)
    
    lidarTest.debugShowPointPosition(landingTarget1)

    lidarTest.client.rotateByYawRateAsync(45, 1, vehicleName).join()
    time.sleep(1)

    lidarTest.executeScan(vehicleName, lidarNames[1:], flightSequence)
    landingTarget2 = lidarTest.detectExactLandingPoint(
        vehicleName + "_" + lidarNames[1] + "_pointcloud.asc",
         vehicleName + "_" + lidarNames[2] + "_pointcloud.asc",
          lidarNames[1:],
           vehicleName)

    lidarTest.debugShowPointPosition(landingTarget2)

    landingTarget = [(landingTarget1[0]+landingTarget2[0])/2,(landingTarget1[1]+landingTarget2[1])/2, (landingTarget1[2] + landingTarget2[2])/2]

    lidarTest.debugShowPointPosition(landingTarget)
    
    #-----------------------
    flightSequence = 3
    print("Sequence 3: landing process starting")
    print("Sequence 3: exactLandingTarget:" +str(landingTarget))
    lidarTest.land(vehicleName, lidarNames, distanceName, landingTarget)
    lidarTest.client.enableApiControl(False)
