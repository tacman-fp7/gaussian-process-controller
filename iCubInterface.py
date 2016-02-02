import numpy as np
import sys
import os
import yarp

class ICubInterface():

    def __init__(self, dataDumperPortName,configFileName, path=''):
        self.isInterfaceLoaded = False
        if(path == ''):
            path = os.path.dirname(os.path.abspath(__file__))
        self.configFileFullName = path +'/'+ configFileName
        self.dataDumperPortName = dataDumperPortName

    def readFloat(self,fileDescriptor,nameString):
        return float(readString(fileDescriptor,nameString))
      
    def readInt(self,fileDescriptor,nameString):
        return int(round(readFloat(fileDescriptor,nameString)))
      
    def readString(self,fileDescriptor,nameString):
        splitline = fileDescriptor.readline().split()
        assert (nameString == splitline[0]),"unexpected entry"
        return splitline[1]
      
    def loadInterfaces(self):

        yarp.Network.init()
        # load parameters from config file
        self.params = dict()
        fileDescriptor = open(self.configFileFullName,'r')
        self.params['robot'] = self.readString(fileDescriptor,'robot')
        self.params['whichHand'] = self.readString(fileDescriptor,'whichHand')
		
        # create, open and connect ports
        # tactile port
        self.tactDataPort = yarp.BufferedPortBottle()
        tactDataPortName = "/gpc/skin/" + self.params['whichHand'] + "_hand_comp:i"
        self.tactDataPort.open(tactDataPortName)
        yarp.Network.connect("/icub/skin/" + self.params['whichHand'] + "_hand_comp",tactDataPortName)
        # encoders port
        self.encDataPort = yarp.BufferedPortBottle()
        encDataPortName = "/gpc/" + self.params['whichHand'] + "_hand/analog:i"
        self.encDataPort.open(encDataPortName)
        yarp.Network.connect("/icub/" + self.params['whichHand'] + "_hand/analog:o",encDataPortName)
        # log port
        self.logPort = yarp.BufferedPortBottle()
        logPortName = "/gpc/log:o"
        self.logPort.open(logPortName)
        yarp.Network.connect(logPortName,self.dataDumperPortName)

        # create driver and options
        self.driver = yarp.PolyDriver()
        options = yarp.Property()
        # set driver options
        options.put("robot",self.params['robot'])
        options.put("device","remote_controlboard")
        options.put("local","/gpc/encoders/" + self.params['whichHand'] + "_arm")
        options.put("remote","/icub/" + self.params['whichHand'] + "_arm")

        # open driver
        print 'Opening motor driver'
        self.driver.open(options)
        if not self.driver.isValid():
            print 'Cannot open driver!'
            sys.exit()        

        # create interfaces
        print 'enabling interfaces'
        self.iPos = self.driver.viewIPositionControl()
        if self.iPos is None:
            print 'Cannot view position interface!'
            sys.exit()
        self.iEnc = self.driver.viewIEncoders()
        if self.iEnc is None:
            print 'Cannot view encoders interface!'
            sys.exit()
        self.iVel = self.driver.viewIVelocityControl()
        if self.iVel is None:
            print 'Cannot view velocity interface!'
            sys.exit()
        self.iCtrl = self.driver.viewIControlMode()
        if self.iCtrl is None:
            print 'Cannot view control mode interface!'
            sys.exit()
        self.iOlc = self.driver.viewIOpenLoopControl()
        if self.iOlc is None:
            print 'Cannot view open loop control mode interface!'
            sys.exit()

        self.numJoints = self.iPos.getAxes()

        ### HEAD ###
        # create driver and options
        self.headDriver = yarp.PolyDriver()
        headOptions = yarp.Property()
        # set driver options
        headOptions.put("robot",self.params['robot'])
        headOptions.put("device","remote_controlboard")
        headOptions.put("local","/gpc/encodersHead/head")
        headOptions.put("remote","/icub/head")
        # open drivers
        self.headDriver.open(headOptions)
        if not self.headDriver.isValid():
            print 'Cannot open head driver!'
            sys.exit()
        # create interfaces
        print 'enabling head interfaces'
        self.iPosHead = self.headDriver.viewIPositionControl()
        if self.iPosHead is None:
            print 'Cannot view head position interface!'
            sys.exit()


        # wait a bit for the interfaces to be ready
        yarp.Time_delay(1.0)

        # read encoders data for the first time
        print 'checking encoders data'
        self.previousEncodersData = yarp.Vector(self.numJoints)
        ret = self.iEnc.getEncoders(self.previousEncodersData.data())
        count = 0
        while ret is False and count < 100:
            yarp.Time_delay(0.01)
            count = count + 1
            ret = self.iEnc.getEncoders(self.previousEncodersData.data())
        if count == 100:
            print 'encoders reading failed'

        # read encoders data from port for the first time
        print 'checking encoders data from port'
        self.previousEncodersDataFP = self.encDataPort.read(False)
        count = 0        
        while self.previousEncodersDataFP is None and count < 100:
            yarp.Time_delay(0.01)
            count = count + 1
            self.previousEncodersDataFP = self.encDataPort.read(False)
        if count == 100:
            print 'encoders data reading from port failed'

        # read tactile data for the first time
        print 'checking tactile data'
        self.previousTactileData = self.tactDataPort.read(False)
        count = 0        
        while self.previousTactileData is None and count < 100:
            yarp.Time_delay(0.01)
            count = count + 1
            self.previousTactileData = self.tactDataPort.read(False)
        if count == 100:
            print 'tactile data reading failed'

        self.isInterfaceLoaded = True

    def readTactileData(self):
        tactBtl = self.tactDataPort.read(False)
        if tactBtl is None:
            tactBtl = self.previousTactileData
        else:
            self.previousTactileData = tactBtl
        return tactBtl
    
    def readEncodersDataFromPort(self):
        encBtl = self.encDataPort.read(False)
        if encBtl is None:
            encBtl = self.previousEncodersDataFP
        else:
            self.previousEncodersDataFP = encBtl
        return encBtl
    

    def readEncodersData(self):
        encodersData = yarp.Vector(self.numJoints)
        ret = self.iEnc.getEncoders(encodersData.data())
        if ret is False:
            encodersData = self.previousEncodersData
        else:
            self.previousEncodersData = encodersData
        return encodersData

    def velocityCommand(self,jointToMove,vel):
        ret = self.iVel.velocityMove(jointToMove,vel)
        if ret is False:
            print 'velocity command failed'

    def openLoopCommand(self,jointToMove,pwm):
        ret = self.iOlc.setRefOutput(jointToMove,pwm)
        if ret is False:
            print 'open loop command failed'

    def stopMoving(self,jointList):
		for i in range(len(jointList)):
			self.iVel.stop(jointList[i])

    def setVelocityMode(self,jointList):
		for i in range(len(jointList)):
			self.iCtrl.setVelocityMode(jointList[i])

    def setPositionMode(self,jointList):
		for i in range(len(jointList)):
			self.iCtrl.setPositionMode(jointList[i])

    def setOpenLoopMode(self,jointList):
		for i in range(len(jointList)):
			self.iCtrl.setOpenLoopMode(jointList[i])

    def setRefAcceleration(self,jointList,refAcceleration):
		for i in range(len(jointList)):
			self.iVel.setRefAcceleration(jointList[i],refAcceleration)

    def setRefVelocity(self,jointList,refVelocity):
		for i in range(len(jointList)):
			self.iPos.setRefAcceleration(jointList[i],refVelocity)

    def setArmPosition(self,encodersList):
        for i in range(len(encodersList)):
            self.iPos.positionMove(i,encodersList[i])
        done = self.iPos.checkMotionDone()
        while not done:
            print "reaching starting position..."        
            yarp.Time_delay(0.1)
            done = self.iPos.checkMotionDone()
        print "ready"
        
    def setHeadPosition(self,encodersList):
        for i in range(len(encodersList)):
            self.iPosHead.positionMove(i,encodersList[i])
        done = self.iPosHead.checkMotionDone()
        while not done:
            print "reaching head starting position..."        
            yarp.Time_delay(0.1)
            done = self.iPosHead.checkMotionDone()
        print "ready"
        

    def setJointPosition(self,joint,position):
        self.iPos.positionMove(joint,position)
        done = self.iPos.checkMotionDone()
        while not done:
            print "reaching target position..."        
            yarp.Time_delay(0.1)
            done = self.iPos.checkMotionDone()
        print "target position reached"
        
    def setHeadJointPosition(self,joint,position):
        self.iPosHead.positionMove(joint,position)
        done = self.iPosHead.checkMotionDone()
        while not done:
            print "reaching head target position..."        
            yarp.Time_delay(0.1)
            done = self.iPosHead.checkMotionDone()
        print "head target position reached"
        

    def logData(self,valuesList):
        bottle = self.logPort.prepare()
        bottle.clear()
        for i in range(len(valuesList)):
            bottle.addDouble(valuesList[i])
        self.logPort.write()

    def closeInterface(self):

        # close the driver and ports
        self.driver.close()
        self.tactDataPort.close()
        self.logPort.close()
        yarp.Network.fini()   
			
