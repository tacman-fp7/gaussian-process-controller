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

def setPosition(handPos,thumbAdductionJointPos,indMidPressuresPercValue,iCubI):
    iCubI.sendDataToGraspModule([1,thumbAdductionJointPos,handPos,indMidPressuresPercValue])
	
def sendAction(handPosIncrement,thumbAdductionJointIncrement,indMidPressuresPercValueIncrement,iCubI):
    iCubI.sendDataToGraspModule([2,thumbAdductionJointIncrement,handPosIncrement,indMidPressuresPercValueIncrement])

def calculateHandPosition(fullEncodersData):
    return (fullEncodersData[13] - fullEncodersData[9])/2 #2F
    #return ((fullEncodersData[13] + fullEncodersData[11])/2 - fullEncodersData[9])/2 #3F

def boundValue(value,minValue,maxValue):
    if value > maxValue:
        return maxValue
    elif value < minValue:
        return minValue
    else:
        return value

def main():

    # module parameters
    maxIterations = [    1000]
    #maxIterations = [    77,    84,   92,    66,    34,    81,    52,    31,     48,    66]

    handStartPos = 20
    #thumbAdductionJointStartPos = 60 #2A
    #indMidPressuresPercStartValue = 0 #3A

    actionEnabled = True

    rolloutsNumFirst = 30
    rolloutsNumStd = 10

    thumbFingerId = 4
    indexFingerId = 0 #3F
    middleFingerId = 1
	
    thumbAdductionJoint = 8

    actionDuration = 0.75
    pauseDuration = 0.0
    tactileAverageTS = 5

    maxHandPos = 20;
    minHandPos = -20; 
    #maxThumbAdductionJointPos = 80; #2A
    #minThumbAdductionJointPos = 40; #2A
    #maxIndMidPressuresPercValue = 30 #3A
    #minIndMidPressuresPercValue = -30 #3A

    dataDumperPortName = "/gpc/log:i"
    iCubIconfigFileName = "iCubInterface.txt"
    inputFilePath = "./"
    initInputFileName = "controller_init_stable_grasp_prox.txt"
    standardInputFileName = "controller.txt"
    outputFilePath = "./"
    outputFileName = "rolloutdata.txt"
    dataPath = "./data/experiments/"

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
    #print expID,isNewExperiment
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
        descriptionData = descriptionData + "handStartPos " + str(handStartPos) + "\n"
        #descriptionData = descriptionData + "thumbAdductionJointStartPos " + str(thumbAdductionJointStartPos) + "\n" #2A
        #descriptionData = descriptionData + "indMidPressuresPercStartValue " + str(indMidPressuresPercStartValue) + "\n" #3A
        descriptionData = descriptionData + "maxHandPos " + str(maxHandPos) + "\n"
        descriptionData = descriptionData + "minHandPos " + str(minHandPos) + "\n"
        #descriptionData = descriptionData + "maxThumbAdductionJointPos " + str(maxThumbAdductionJointPos) + "\n" #2A
        #descriptionData = descriptionData + "minThumbAdductionJointPos " + str(minThumbAdductionJointPos) + "\n" #2A
        #descriptionData = descriptionData + "maxIndMidPressuresPercValue " + str(maxIndMidPressuresPercValue) + "\n" #3A
        #descriptionData = descriptionData + "minIndMidPressuresPercValue " + str(minIndMidPressuresPercValue) + "\n" #3A
        descriptionData = descriptionData + "actionDuration " + str(actionDuration) + "\n"
        descriptionData = descriptionData + "pauseDuration " + str(pauseDuration) + "\n"
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

    # load gaussian process controller
    gp = gpc.GPController(inputFileFullName)
    gp.load_controller()

    # load iCub interface
    iCubI = iCubInterface.ICubInterface(dataDumperPortName,iCubIconfigFileName)
    iCubI.loadInterfaces()
    # set start position
    if actionEnabled:
	setPosition(handStartPos,0,0,iCubI) #1A
	#setPosition(handStartPos,thumbAdductionJointStartPos,0,iCubI) #2A
	#setPosition(handStartPos,thumbAdductionJointStartPos,indMidPressuresPercStartValue,iCubI) #3A
        time.sleep(4)

    # wait for the user
    raw_input("- press enter to start the controller -")

    fd = open(outputFileFullName,"w")
    fd.write("nrollouts: ")
    fd.write(str(rolloutsNum))
    fd.write("\n")
    fd.close()

    rolloutsCounter = 0
    while rolloutsCounter < rolloutsNum:

        print "starting iteration n. ",rolloutsCounter + 1
        fd = open(outputFileFullName,"a")
        fd.write("# HEADER ")
        fd.write(str(rolloutsCounter + 1))
        fd.write("\n")

        iterCounter = 0
        exit = False
        positionArray = [0] #1A
        oldPositionArray = [0] #1A
        newPositionArray = [0] #1A
        #positionArray = [0,0] #2A
        #oldPositionArray = [0,0] #2A
        #newPositionArray = [0,0] #2A
        #positionArray = [0,0,0] #3A
        #oldPositionArray = [0,0,0] #3A
        #newPositionArray = [0,0,0] #3A

        #TEMPORARY CODE
        accuracyList = [[]] #1A
        #accuracyList = [[],[]] #2-3A
		
        oldContactPositions = np.zeros(6)
        contactPositionsDiffAvg = np.zeros(6)		
        contactPositionsMatrix = np.array([])
		
        # main loop
        while iterCounter < maxIterations[rolloutsCounter%10] and not exit:

            tactileDataAverage = np.zeros(36)
            for i in range(tactileAverageTS):
			
                # read tactile data
                fullTactileData = iCubI.readTactileData()
                tactileData = []              
                for j in range(12):
                    tactileData.append(fullTactileData.get(12*thumbFingerId+j).asDouble())
                for j in range(12): #3F
                    tactileData.append(fullTactileData.get(12*indexFingerId+j).asDouble()) #3F
                for j in range(12):
                    tactileData.append(fullTactileData.get(12*middleFingerId+j).asDouble())

                #if iterCounter%10 == 0:
                #    print 'raw',tactileData[0],tactileData[1]
                tactileDataAr = np.array(tactileData)
                tactileDataAverage = tactileDataAverage + tactileDataAr
                #if iterCounter%10 == 0:
                #    print 'avg',tactileDataAverage[0],tactileDataAverage[1]	
                time.sleep(0.02)
			
            tactileDataAverage = tactileDataAverage / tactileAverageTS
            tactileData = tactileDataAverage.tolist()
			
            #if iterCounter%10 == 0:
            #    print 'end',tactileData[0],tactileData[1]			
				
            contactPositions = []
            contactPositions.extend(util.getContactPosition(tactileData[0:12]))
            contactPositions.extend(util.getContactPosition(tactileData[12:24]))
            contactPositions.extend(util.getContactPosition(tactileData[24:36])) #3F
            print 'CP',contactPositions

            if len(contactPositionsMatrix) == 0:
                contactPositionsMatrix = np.array([contactPositions])
            else:
                contactPositionsMatrix = np.append(contactPositionsMatrix,[contactPositions],0)
			
            if iterCounter > 0:
                contactPositionsDiffAvg = contactPositionsDiffAvg + abs(oldContactPositions - np.array(contactPositions))			
                #if iterCounter%10 == 0:
                    #print 'old',oldContactPositions			
                    #print 'now',np.array(contactPositions)
                    #print 'avg',contactPositionsDiffAvg			
            oldContactPositions = np.array(contactPositions)
				
            fullEncodersData = iCubI.readEncodersData()
            positionArray[0] = calculateHandPosition(fullEncodersData)
            #positionArray[1] = fullEncodersData.get(thumbAdductionJoint) #2A
            #positionArray[2] = newPositionArray[2] #3A

            #TEMPORARY CODE
            expectedMovement = [0] #1A
            actualMovement = [0] #1A
            movementAccuracyRate = [0] #1A
            #expectedMovement = [0,0] #2-3A
            #actualMovement = [0,0] #2-3A
            #movementAccuracyRate = [0,0] #2-3A
            expectedMovement[0] = newPositionArray[0]-oldPositionArray[0]
            #expectedMovement[1] = newPositionArray[1]-oldPositionArray[1] #2-3A
            actualMovement[0] = positionArray[0]-oldPositionArray[0]
            #actualMovement[1] = positionArray[1]-oldPositionArray[1] #2-3A
            if expectedMovement[0] != 0:
               movementAccuracyRate[0] = 100.0*actualMovement[0]/expectedMovement[0]
            #if expectedMovement[1] != 0: #2-3A
            #   movementAccuracyRate[1] = 100.0*actualMovement[1]/expectedMovement[1] #2-3A
            #print "{:7.3f}".format(oldPositionArray[0]),'\t',"{:7.3f}".format(expectedMovement[0]),'\t',"{:7.3f}".format(actualMovement[0]),'\t',"{:6.2f}".format(movementAccuracyRate[0])
            #print "{:7.3f}".format(oldPositionArray[1]),'\t',"{:7.3f}".format(expectedMovement[1]),'\t',"{:7.3f}".format(actualMovement[1]),'\t',"{:6.2f}".format(movementAccuracyRate[1])
            #print "---"
            if abs(expectedMovement[0]) > 1:
               accuracyList[0].append(movementAccuracyRate[0])
            #if abs(expectedMovement[1]) > 2: #2-3A
            #   accuracyList[1].append(movementAccuracyRate[1]) #2-3A

            oldPositionArray[0] = positionArray[0]
            #oldPositionArray[1] = positionArray[1] #2-3A

            state = [tactileData,contactPositions]

            # choose action
            action = gp.get_control_nonoise(state)

            # update and cut distal joints position
            newPositionArray[0] = boundValue(positionArray[0] + action[0],minHandPos,maxHandPos)
            #newPositionArray[1] = boundValue(positionArray[1] + action[1],minThumbAdductionJointPos,maxThumbAdductionJointPos) #2A
            #newPositionArray[2] = boundValue(positionArray[2] + action[2],minIndMidPressuresPercValue,maxIndMidPressuresPercValue) #3A

            #print newPositionArray	
			
            # apply action
            if actionEnabled:
                sendAction(newPositionArray[0],0,0,iCubI) #1A
                #sendAction(newPositionArray[0],newPositionArray[1],0,iCubI) #2A
                #sendAction(newPositionArray[0],newPositionArray[1],newPositionArray[2],iCubI) #3A

            beforeTS = time.time()
            # here processing can take place 
            afterTS = time.time()
            timeForAverage = tactileAverageTS*0.02
            timeToSleep = max((actionDuration-(afterTS-beforeTS))-timeForAverage,0)
            time.sleep(timeToSleep)
			
            # wait for stabilization
            time.sleep(pauseDuration)

            # log data
            iCubI.logData([contactPositions[0] + contactPositions[4] - 3.25])
            logArray(tactileData,fd)
            #logArray([fullEncodersData[9],[fullEncodersData[13]],fd) #2F
            logArray([fullEncodersData[9],fullEncodersData[11],fullEncodersData[13]],fd) #3F
            logArray(contactPositions,fd)
            logArray(action,fd)
            logArray([0],fd) #reward
            fd.write("\n")

            iterCounter = iterCounter + 1
            exit = False #exitModule(resetProbability)

        fd.close()

        #TEMPORARY CODE
        #print accuracyList[0]
        print '---'
        print 'thumb','\t',np.mean(accuracyList[0]),'\t',np.std(accuracyList[0]),'\t',len(accuracyList[0])
        #print accuracyList[1] #2-3A
        #print 'hand','\t',np.mean(accuracyList[1]),'\t',np.std(accuracyList[1]),'\t',len(accuracyList[1]) #2-3A
        print 'avgContactDiff',contactPositionsDiffAvg/(iterCounter-1)
		
	contactPositionsMedian = []
	for i in range(6):
	    tempSum = []
	    currContactPositionsArray = np.array([row[i] for row in contactPositionsMatrix])
            for j in range(len(currContactPositionsArray) - 1):
               for k in range(j + 1,len(currContactPositionsArray)):
                  tempSum.append(abs(currContactPositionsArray[j] - currContactPositionsArray[k]))
            contactPositionsMedian.append(np.median(tempSum))
        print 'med',contactPositionsMedian
        print '---'

        if actionEnabled:
            print "hand ripositioning..."
            setPosition(handStartPos,0,0,iCubI) #1A
            #setPosition(handStartPos,thumbAdductionJointStartPos,0,iCubI) #2A
            #setPosition(handStartPos,thumbAdductionJointStartPos,indMidPressuresPercStartValue,iCubI) #3A
            time.sleep(4)
			
            print "...done"


        rolloutsCounter = rolloutsCounter + 1
            
    # copy input and output file
    os.system("cp " + inputFileFullName + " " + backupInputFileFullName)
    os.system("cp " + outputFileFullName + " " + backupOutputFileFullName)

    iCubI.closeInterface()
    
		
if __name__ == "__main__":
    main()
