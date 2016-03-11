import gp_controller as gpc
import iCubInterface
import iCubUtility as util
import numpy as np
import yarp
import time
import math
import random
import os
import sys
#import  as find_lines

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


def main():

    # module parameters
    maxIterations = [    50]
    #maxIterations = [    77,    84,   134,    66,    34,    81,    52,    31,     48,    66]

    thumbDistalJointStartPos = 15
    indexDistalJointStartPos = 15
    middleDistalJointStartPos = 15

    actionEnabled = True

    rolloutsNumFirst = 30
    rolloutsNumStd = 10

    thumbFingerId = 4
    indexFingerId = 0
    middleFingerId = 1
	
    thumbDistalJoint = 10
    indexDistalJoint = 12
    middleDistalJoint = 14

    actionDuration = 2
    pauseDuration = 0.0

#    normalizedMaxVoltageY = 1.0
#    maxVoltageProxJointY = 250.0
#    maxVoltageDistJointY = 800.0
#    slopeAtMaxVoltageY = 1.0

    maxThumbDistalPos = 60;
    minThumbDistalPos = 0; 
    maxDistalPos = 90;
    minDistalPos = 0; 

    dataDumperPortName = "/gpc/log:i"
    iCubIconfigFileName = "iCubInterface.txt"
    inputFilePath = "./"
    initInputFileName = "controller_init_stable_grasp_dist.txt"
    standardInputFileName = "controller_input.txt"
    outputFilePath = "./"
    outputFileName = "controller_output.txt"
    dataPath = "./data/experiments/"
  
#    jointsToActuate = [thumbDistalJoint,indexDistalJoint,middleDistalJoint]
    
    fileNameIterID = "iterationID.txt"
    fileNameExperimentID = "experimentID.txt"
    fileNameExpParams = "parameters.txt"

    isNewExperiment = False
    if len(sys.argv) > 1:
        if sys.argv[1] == 'new':
            isNewExperiment = True 
    
    expID = readValueFromFile(fileNameExperimentID)        
    if isNewExperiment:
        expID = expID + 1
        writeIntoFile(fileNameExperimentID,str(expID))

    # create output folder name
    experimentFolderName = dataPath + "exp_" + str(expID) + "/" # could be changed adding more information about the experiment
    print expID,isNewExperiment
    if os.path.exists(experimentFolderName):
        # get iteration ID
        iterID = readValueFromFile(fileNameIterID)
        writeIntoFile(fileNameIterID,str(iterID+1))
        inputFileFullName = inputFilePath + standardInputFileName
        rolloutsNum = rolloutsNumStd
    else:
        # create directory, create an experiment descrition file and reset iteration ID
        os.mkdir(experimentFolderName)
        descriptionData = ""
        descriptionData = descriptionData + "thumbDistalJointStartPos " + str(thumbDistalJointStartPos) + "\n"
        descriptionData = descriptionData + "indexDistalJointStartPos " + str(indexDistalJointStartPos) + "\n"
        descriptionData = descriptionData + "middleDistalJointStartPos " + str(middleDistalJointStartPos) + "\n"
        descriptionData = descriptionData + "actionDuration " + str(actionDuration) + "\n"
        descriptionData = descriptionData + "pauseDuration " + str(pauseDuration) + "\n"
        #descriptionData = descriptionData + "finger " + str(finger) + "\n"
        #descriptionData = descriptionData + "jointActuated " + str(proximalJoint) + " " + str(distalJoint) + "\n"
        #descriptionData = descriptionData + "jointStartingPositions " + str(proximalJointStartPos) + " " + str(distalJointStartPos) + "\n"
        #descriptionData = descriptionData + "resetProbabilty " + str(resetProbability) + "\n"
        #descriptionData = descriptionData + "additionaNotes " + "" + "\n"
        writeIntoFile(experimentFolderName + fileNameExpParams,descriptionData)
        iterID = 0
        writeIntoFile(fileNameIterID,"1")
        inputFileFullName = inputFilePath + initInputFileName
        rolloutsNum = rolloutsNumFirst

    outputInputFileSuffix = str(expID) + "_" + str(iterID);
    backupOutputFileFullName = experimentFolderName + "contr_out_" + outputInputFileSuffix + ".txt"
    backupInputFileFullName = experimentFolderName + "contr_in_" + outputInputFileSuffix + ".txt"
    outputFileFullName = outputFilePath + outputFileName

    # calculate voltageX-voltageY mapping parameters (voltageY = k*(voltageX^(1/3)))
#    k = pow(3*slopeAtMaxVoltageY*(pow(normalizedMaxVoltageY,2)),(1/3.0))

#    maxVoltageX = pow(normalizedMaxVoltageY/k,3)

    # load gaussian process controller
    gp = gpc.GPController(inputFileFullName)
    gp.load_controller()

    # load iCub interface
    iCubI = iCubInterface.ICubInterface(dataDumperPortName,iCubIconfigFileName)
    iCubI.loadInterfaces()

    # set start position
    if actionEnabled:
        iCubI.setJointPositionNoWait(thumbDistalJoint,thumbDistalJointStartPos)
        #iCubI.setJointPosition(indexDistalJoint,indexDistalJointStartPos) #3FING
        iCubI.setJointPositionNoWait(middleDistalJoint,middleDistalJointStartPos)
        time.sleep(3)

    # wait for the user
    raw_input("- press enter to start the controller -")

    fd = open(outputFileFullName,"w")
    fd.write("nrollouts: ")
    fd.write(str(rolloutsNum))
    fd.write("\n")
    fd.close()
    
    # initialize velocity mode
#    if actionEnabled:
#        iCubI.setOpenLoopMode(jointsToActuate)

    rolloutsCounter = 0
    while rolloutsCounter < rolloutsNum:

        print "starting iteration n. ",rolloutsCounter + 1
        fd = open(outputFileFullName,"a")
        fd.write("# HEADER ")
        fd.write(str(rolloutsCounter + 1))
        fd.write("\n")

        iterCounter = 0
        exit = False
        distalJointsPos = [0,0] #2FING
        oldDistalJointsPos = [0,0] #2FING
        newDistalJointsPos = [0,0] #2FING
        #distalJointsPos = [0,0,0] #3FING
        #oldDistalJointsPos = [0,0,0] #3FING
        #newDistalJointsPos = [0,0,0] #3FING

        #TEMPORARY CODE
        accuracyList = [[],[]]

#        voltage = [0,0]
#        oldVoltage = [0,0]
#        realVoltage = [0,0]

        # main loop
        while iterCounter < maxIterations[rolloutsCounter%10] and not exit:

            # read tactile data
            fullTactileData = iCubI.readTactileData()
            tactileData = []              
            for j in range(12):
                tactileData.append(fullTactileData.get(12*thumbFingerId+j).asDouble())
            #for j in range(12):
            #    tactileData.append(fullTactileData.get(12*indexFingerId+j).asDouble()) #3FING
            for j in range(12):
                tactileData.append(fullTactileData.get(12*middleFingerId+j).asDouble())
				
            contactPositions = []
            contactPositions.append(util.getContactPosition(tactileData[0:12]))
            contactPositions.append(util.getContactPosition(tactileData[12:24]))
            #contactPositions.append(util.getContactPosition(tactileData[24:36])) #3FING
			
            fullEncodersData = iCubI.readEncodersData()
            distalJointsPos[0] = fullEncodersData.get(thumbDistalJoint)
            distalJointsPos[1] = fullEncodersData.get(middleDistalJoint) #2FING
            #distalJointsPos[1] = fullEncodersData.get(indexDistalJoint) #3FING
            #distalJointsPos[2] = fullEncodersData.get(middleDistalJoint) #3FING

            #TEMPORARY CODE
            expectedMovement = [0,0]
            actualMovement = [0,0]
            movementAccuracyRate = [0,0]
            expectedMovement[0] = newDistalJointsPos[0]-oldDistalJointsPos[0]
            expectedMovement[1] = newDistalJointsPos[1]-oldDistalJointsPos[1]
            actualMovement[0] = distalJointsPos[0]-oldDistalJointsPos[0]
            actualMovement[1] = distalJointsPos[1]-oldDistalJointsPos[1]
            if expectedMovement[0] != 0:
               movementAccuracyRate[0] = 100.0*actualMovement[0]/expectedMovement[0]
            if expectedMovement[1] != 0:
               movementAccuracyRate[1] = 100.0*actualMovement[1]/expectedMovement[1]
            print "{:7.3f}".format(oldDistalJointsPos[0]),'\t',"{:7.3f}".format(expectedMovement[0]),'\t',"{:7.3f}".format(actualMovement[0]),'\t',"{:6.2f}".format(movementAccuracyRate[0])
            print "{:7.3f}".format(oldDistalJointsPos[1]),'\t',"{:7.3f}".format(expectedMovement[1]),'\t',"{:7.3f}".format(actualMovement[1]),'\t',"{:6.2f}".format(movementAccuracyRate[1])
            print "---"
            if abs(expectedMovement[0]) > 5:
               accuracyList[0].append(movementAccuracyRate[0])
            if abs(expectedMovement[1]) > 5:
               accuracyList[1].append(movementAccuracyRate[1])



            oldDistalJointsPos[0] = distalJointsPos[0]
            oldDistalJointsPos[1] = distalJointsPos[1]


            state = [tactileData,contactPositions]

            # choose action
            action = gp.get_control(state)

            # update and cut distal joints position
            newDistalJointsPos[0] = distalJointsPos[0] + action[0]
            newDistalJointsPos[1] = distalJointsPos[1] + action[1]
            #newDistalJointsPos[2] = distalJointsPos[2] + action[1] #3FING
            if newDistalJointsPos[0] > maxThumbDistalPos:
                newDistalJointsPos[0] = maxThumbDistalPos
            if newDistalJointsPos[0] < minThumbDistalPos:
                newDistalJointsPos[0] = minThumbDistalPos
            if newDistalJointsPos[1] > maxDistalPos:
                newDistalJointsPos[1] = maxDistalPos
            if newDistalJointsPos[1] < minDistalPos:
                newDistalJointsPos[1] = minDistalPos
            #if newDistalJointsPos[2] > maxDistalPos: #3FING
            #    newDistalJointsPos[2] = maxDistalPos #3FING
            #if newDistalJointsPos[2] < minDistalPos: #3FING
            #    newDistalJointsPos[2] = minDistalPos #3FING
			
            #if abs(voltage[0]) > maxVoltageX:
            #    voltage[0] = maxVoltageX*np.sign(voltage[0])
            #if abs(voltage[1]) > maxVoltageX:
            #    voltage[1] = maxVoltageX*np.sign(voltage[1])

            # calculate real applied voltage
#            realVoltage[0] = maxVoltageProxJointY*k*pow(abs(voltage[0]),1/3.0)*np.sign(voltage[0])
#            realVoltage[1] = maxVoltageDistJointY*k*pow(abs(voltage[1]),1/3.0)*np.sign(voltage[1])

            # voltage safety check (it should never happen!)
            #if abs(realVoltage[0]) > maxVoltageProxJointY:
            #    realVoltage[0] = maxVoltageProxJointY*np.sign(realVoltage[0])
            #    print 'warning, voltage out of bounds!'
            #if abs(realVoltage[1]) > maxVoltageDistJointY:
            #    realVoltage[1] = maxVoltageDistJointY*np.sign(realVoltage[1])
            #    print 'warning, voltage out of bounds!'


            # apply action
            if actionEnabled:
                iCubI.setJointPositionNoWait(thumbDistalJoint,newDistalJointsPos[0])
                iCubI.setJointPositionNoWait(middleDistalJoint,newDistalJointsPos[1]) #2FING
                #iCubI.setJointPositionNoWait(indexDistalJoint,newDistalJointsPos[1]) #3FING
                #iCubI.setJointPositionNoWait(middleDistalJoint,newDistalJointsPos[2]) #3FING
#                iCubI.openLoopCommand(proximalJoint,realVoltage[0])        
#                iCubI.openLoopCommand(distalJoint,realVoltage[1])

            # get feedback angle
#            previousFbAngle = currentFbAngle
            beforeTS = time.time()
           # if rolloutsCounter == 0 and iterCounter < 50:
           # matplotlib.image.imsave('images/test_'+ str(rolloutsCounter) + '_' + str(iterCounter) +'.tiff', img_array, format='tiff')
#            currentFbAngle = getFeedbackAngle(yarp_image,img_array)
#            fbAngleDifference = calculateFeedbackAngleDifference(previousFbAngle,currentFbAngle,fbAngleRange)
#            if abs(fbAngleDifference) > maxFbAngleDifference:
#                currentFbAngle = previousFbAngle
#                fbAngleDifference = 0.0
#            print fbAngleDifference
            afterTS = time.time()
            timeToSleep = max(actionDuration-(afterTS-beforeTS),0)
            time.sleep(timeToSleep)


            #print "curr ",previousFbAngle*180/3.1415,"diff ",fbAngleDifference*180/3.1415,afterTS - beforeTS,timeToSleep


            # wait for stabilization
            time.sleep(pauseDuration)

            # log data
            iCubI.logData(tactileData + contactPositions[0] + contactPositions[1] + [action[0],action[1]])#[action[0],action[1]])
            logArray(tactileData,fd)
            logArray(action,fd)
            fd.write("\n")

            iterCounter = iterCounter + 1
            exit = False #exitModule(resetProbability)

        fd.close()

        #TEMPORARY CODE
        print accuracyList[0]
        print 'thumb','\t',np.mean(accuracyList[0]),'\t',np.std(accuracyList[0]),'\t',len(accuracyList[0])
        print accuracyList[1]
        print 'middle','\t',np.mean(accuracyList[1]),'\t',np.std(accuracyList[1]),'\t',len(accuracyList[1])

        if actionEnabled:
            print "finger ripositioning..."
            # finger repositioning
            iCubI.setJointPositionNoWait(thumbDistalJoint,thumbDistalJointStartPos)
            #iCubI.setJointPosition(indexDistalJoint,indexDistalJointStartPos) #3FING
            iCubI.setJointPositionNoWait(middleDistalJoint,middleDistalJointStartPos)
            time.sleep(3)

#            iCubI.setPositionMode(jointsToActuate)
#            iCubI.setJointPosition(proximalJoint,0.0)
#            iCubI.setJointPosition(distalJoint,0.0)
#            time.sleep(waitTimeForFingersRepositioning)
#            iCubI.setJointPosition(proximalJoint,proximalJointStartPos)
#            iCubI.setJointPosition(distalJoint,distalJointStartPos)
#            time.sleep(waitTimeForFingersRepositioning)
#            iCubI.setOpenLoopMode(jointsToActuate)
            print "...done"


        rolloutsCounter = rolloutsCounter + 1
            
    os.system("cp " + inputFileFullName + " " + backupInputFileFullName)
    os.system("cp " + outputFileFullName + " " + backupOutputFileFullName)

    # copy input and output file
    # restore position mode and close iCubInterface
#    if actionEnabled:
#        iCubI.setPositionMode(jointsToActuate)
    iCubI.closeInterface()
    
		
if __name__ == "__main__":
    main()
