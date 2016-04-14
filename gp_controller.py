import numpy as np
import math
import time
import os


def exponentialSquaredKernel(a, b, params):
  if(type(params['bandwidth']) == type([])):
    Q = np.diag(map(lambda x: 1.0/(x**2), params['bandwidth']))
  else:
    Q = 1.0/(params['bandwidth']**2)

  return params['scale'] * math.exp(-0.5 * np.dot(np.dot(a-b, Q),a-b))
   
class GPController():

    def __init__(self, controllerFileName, path=''):
      self.isControllerLoaded = False
      if(path == ''):
        path = os.path.dirname(os.path.abspath(__file__))
      self.controllerParamsFile = path +'/'+controllerFileName
      
    def readFloat(self, filedescriptor, namestring):
      splitline = filedescriptor.readline().split()
      assert (namestring == splitline[0]),"unexpected entry"
      return float(splitline[1])
      
    def readInt(self, filedescriptor, namestring):
      splitline = filedescriptor.readline().split()
      assert (namestring == splitline[0]),"unexpected entry"
      return int(round(float(splitline[1])))
      
    def load_controller(self):  
      #load hyperparameters  
      self.hyperparameters = dict();
      f = open(self.controllerParamsFile,'r')
      nparams = self.readInt(f, 'nparams')
      for idx in range(0, nparams):
        splitline = f.readline().split()
        val = map(lambda x: float(x), splitline[1:])
        if len(val) == 1:
          val = val[0] 
        self.hyperparameters[splitline[0]] = val
      #load input and output data, chol
      npoints = self.readInt(f, 'npoints')
      inputlist = list()
      for i in range(0, npoints):
        splitline = f.readline().split(',')
        inputlist.append(map(lambda x: float(x), splitline))
      self.inputdata = np.asarray(inputlist)
      alpha = list()  
      for i in range(0, npoints):
        splitline = f.readline().split(',')
        alpha.append(map(lambda x: float(x), splitline))    
      self.alpha = np.asarray(alpha)
      cholKy = list()
      for i in range(0, npoints):
        splitline = f.readline().split(',')
        cholKy.append(map(lambda x: float(x), splitline)) 
      self.cholKy = np.asarray(cholKy)    
      f.close()
      print "in: ", self.inputdata.shape, " alpha: ", self.alpha.shape, " cholKy ", self.cholKy.shape
      self.isControllerLoaded = True
    
    def get_features(self,state):
        f = np.zeros([1]) #2F
        #f = np.zeros([6]) #3F
        tactileData = state[0]
        contactPositions = state[1]

        f[0] = contactPositions[0] + contactPositions[4]
        #f[0] = contactPositions[0]
        #f[1] = contactPositions[1]
        #f[2] = contactPositions[4]
        #f[3] = contactPositions[5]
        #f[4] = contactPositions[4] #3F
        #f[5] = contactPositions[5] #3F
        return f
    
    def get_control(self,state):
      #get kernel-based control
      if(not self.isControllerLoaded):
        return(0,0)
      if(self.inputdata.shape[0] == 0):
        stdev = self.hyperparameters['stdevInit']
        mean = np.zeros(np.shape(stdev))
      else:
        f = self.get_features(state)
        pred = self.get_mu_sigma(f)
        mean = pred[0]
        stdev = pred[1]
      eps = np.random.randn(np.size(mean))  
      u = mean + stdev * eps
      clippedu = np.minimum(np.maximum(u,self.hyperparameters['minvalues']), self.hyperparameters['maxvalues'])
      return clippedu
      
    def get_control_nonoise(self, state):
      f = self.get_features(state)
      pred = self.get_mu_sigma(f)
      u = pred[0]
      clippedu = np.minimum(np.maximum(u,self.hyperparameters['minvalues']), self.hyperparameters['maxvalues'])
      return clippedu      
    
    def get_mu_sigma(self, features):
      #print features
      kvec = map(lambda x: exponentialSquaredKernel(x, features, self.hyperparameters) ,self.inputdata )
      mean = np.dot(kvec, self.alpha)
      #print mean
      temp = np.linalg.solve(self.cholKy.transpose(),kvec)
      c = exponentialSquaredKernel(features, features, self.hyperparameters) + self.hyperparameters['lambda']
      covariance = c - np.dot(temp.transpose(), temp)
      if covariance < 0:
        covariance = 0
      stdev = np.sqrt(covariance) 
      return [mean, stdev]
      

