import gp_controller as gpc
import iCubInterface
import numpy as np
import yarp
import time
import math
import random
import os
import find_lines

def exitModule(resetProbability):
    randomNum = random.random()
    if randomNum < resetProbability:
        return True
    return False

def logArray(array,fd):
    for i in range(len(array)):
        fd.write(str(array[i]))
        fd.write(" ")

def readValueFromFile(fileName):
    fd = open(fileName,"r")
    line = fd.readline().split()
    value = int(line[0])
    fd.close()
    return value

def writeIntoFile(fileName,string):
    fd = open(fileName,"w")
    fd.write(string)
    fd.close()

def addDescriptionData(dataString,parameter,value):
    dataString = dataString + parameter + " " + value + "\n"

def readImage(cameraPort,yarp_image):
    cameraPort.read(yarp_image)

def getFeedbackAngle(yarp_image,img_array):

    img_bgr = img_array[:,:,[2,1,0]]
    retList = find_lines.run_system(img_bgr)
    first_line_polar = retList[0]
    second_line_polar = retList[1]
    first_theta = first_line_polar[1]
    second_theta = second_line_polar[1]
    return [first_theta,second_theta]

def calculateFeedbackAngleDifference(previousFbAngle,currentFbAngle,fbAngleRange):

    delta = currentFbAngle - previousFbAngle

    if abs(delta) < fbAngleRange/2.0:
        fbAngleDifference = delta
    else:
        fbAngleDifference = np.sign(-delta)*(fbAngleRange - abs(delta))

    return fbAngleDifference

def findNewAngle(angle,alpha,beta):

    p1 = [1.0,0.0,0.0]
    p2 = [math.cos(angle),math.sin(angle),0.0]

    rx = [[1,0,0],[0,math.cos(alpha),-math.sin(alpha)],[0,math.sin(alpha),math.cos(alpha)]]
    ry = [[math.cos(beta),0,-math.sin(beta)],[0,1,0],[math.sin(beta),0,math.cos(beta)]]

    np1 = np.dot(ry,np.dot(rx,p1))
    np2 = np.dot(ry,np.dot(rx,p2))

    v1 = [np1[0],np1[1]]
    v2 = [np2[0],np2[1]]

    cosang = np.dot(v1,v2)
    sinang = np.linalg.norm(np.cross(v1,v2))
    newAngle = np.arctan2(sinang,cosang)
    return newAngle

def main():

    # module parameters
    maxIterations = [    77,    14,   134,    66,    10,    81,    22,    31,     3,    66]
    maxIterations2 = [    50,    14,   134,    66,    10,    81,    22,    31,     3,    66]

    proximalJointStartPos = 40
    distalJointStartPos = 0
    joint1StartPos = 18
    #                    0               1   2   3   4   5   6   7   8   9  10  11  12                    13                  14  15
    startingPosEncs = [-44, joint1StartPos, -4, 39,-14,  2,  2, 18, 12, 20,163,  0,  0,proximalJointStartPos,distalJointStartPos,  0]   
    
    actionEnabled = True

    rolloutsNum = 100

    finger = 1
    proximalJoint = 13
    distalJoint = 14
    proximalJointEnc = 6
    distalJointEnc_1 = 7
    distalJointEnc_2 = 8

    resetProbability = 0.02

    actionDuration = 0.1
    pauseDuration = 0.0

    maxFbAngle = math.pi
    minFbAngle = 0
    maxFbAngleDifference = math.pi/3.0
    fbAngleRange = maxFbAngle - minFbAngle

    normalizedMaxVoltageY = 1.0
    maxVoltageProxJointY = 250.0
    maxVoltageDistJointY = 600.0
    slopeAtMaxVoltageY = 1.0

    waitTimeForFingersRepositioning = 7.0

    dataDumperPortName = "/gpc/log:i"
    iCubIconfigFileName = "iCubInterface.txt"
    inputFilePath = "./"
    initInputFileName = "controller_init.txt"
    standardInputFileName = "controller_input.txt"
    outputFilePath = "./"
    outputFileName = "controller_output.txt"
    dataPath = "./data/experiments/"
  
    jointsToActuate = [proximalJoint,distalJoint]
    
    fileNameIterID = "iterationID.txt"
    fileNameExpParams = "parameters.txt"

    # create output folder name
    expID = 16
    experimentFolderName = dataPath + "exp_" + str(expID) + "/" # could be changed adding more information about the experiment



    # load iCub interface
    iCubI = iCubInterface.ICubInterface(dataDumperPortName,iCubIconfigFileName)
    iCubI.loadInterfaces()

    # cameras port
    cameraPort = yarp.Port()
    cameraPortName = "/gpc/leftEye"
    cameraPort.open(cameraPortName)
    yarp.Network.connect("/icub/cam/left",cameraPortName)

    # image setting
    width = 640
    height = 480 
    # Create numpy array to receive the image and the YARP image wrapped around it
    img_array = np.zeros((height, width, 3), dtype=np.uint8)
    yarp_image = yarp.ImageRgb()
    yarp_image.resize(width, height)
    yarp_image.setExternal(img_array, img_array.shape[1], img_array.shape[0])


    # wait for the user
    raw_input("- press enter to start the controller -")

    readImage(cameraPort,yarp_image)
    bothCurrentFbAngle = getFeedbackAngle(yarp_image,img_array)
    currentFbAngle = bothCurrentFbAngle[0]
    rolloutsCounter = 0
    while rolloutsCounter < rolloutsNum:

        # get feedback angle
        previousFbAngle = currentFbAngle
        beforeTS = time.time()
        readImage(cameraPort,yarp_image)
        bothCurrentFbAngle = getFeedbackAngle(yarp_image,img_array)
        currentFbAngle = bothCurrentFbAngle[0]
        fbAngleDifference = calculateFeedbackAngleDifference(previousFbAngle,currentFbAngle,fbAngleRange)
#        print previousFbAngle*180.0/3.1415,currentFbAngle*180.0/3.1415,fbAngleDifference*180.0/3.1415
        na1 = findNewAngle(bothCurrentFbAngle[0],0,0)
        na2 = findNewAngle(bothCurrentFbAngle[1],0,0)
        print bothCurrentFbAngle[0]*180.0/3.1415,bothCurrentFbAngle[1]*180.0/3.1415,(bothCurrentFbAngle[0]-bothCurrentFbAngle[1])*180.0/3.1415,na1*180.0/3.1415,na2*180.0/3.1415,(na2-na1)*180.0/3.1415
        if abs(fbAngleDifference > maxFbAngleDifference):
            currentFbAngle = previousFbAngle
            fbAngleDifference = 0.0
        afterTS = time.time()
        timeToSleep = max(actionDuration-(afterTS-beforeTS),0)
        time.sleep(timeToSleep)

#        print "curr ",previousFbAngle*180/3.1415,"diff ",fbAngleDifference*180/3.1415,afterTS - beforeTS,timeToSleep

        exit = False #exitModule(resetProbability)

        rolloutsCounter = rolloutsCounter + 1
            
    # copy input and output file
    # restore position mode and close iCubInterface
    cameraPort.close()
    iCubI.closeInterface()
    
		
if __name__ == "__main__":
    main()
