#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
this class is for EnKF operations

@author: cesny
"""
import numpy as np
from utils import setCTMVehicles, forwardCTMPropagation, CTMcreateInitialEnsemble, VmaxCreateInitialEnsemble, VmaxtoCritDen, cellToLength, lengthToCell


class EnKF:
  '''
  this class is used to implement EnKF operations
  for implementation details, this code follows Evensen2003 
  '''
  def __init__(self, obsError, modelError, sampleSize, stateDim, obsDim, m=None, assimilatedDensities=None, H=None, EnKFtype='CTM', nonLinearObs=False, droneLoc = None, trafficNet=None, droneDenObsError=None):
    self.obsError = obsError  # specifies standard dev. of observ. white noise
    self.modelError = modelError  # specifies standard dev. of model white noise
    self.sampleSize = sampleSize  # number of ensemble members
    self.stateDim = stateDim  # dimension of an ensemble member
    self.obsDim = obsDim  # dimension of observation vector
    self.EnKFtype = EnKFtype  # could be 'CTM' or 'Vmax'
    self.nonLinearObs = nonLinearObs
    self.assimDen = assimilatedDensities  # a list with two elements, assim. den upstream and assim den downstream
    self.m = m  # this is a function that takes as input the state vector as a list [vmax1, vmax2] and returns the model prediction of the parameters [v1, v2]
    self.droneLoc = droneLoc  # stores the drone location (linkID, cell) tuple
    self.H = H  # create the matrix (vector) H if it is available
    self.locToCell = dict()  # maps the tuple (link, cell) to cell between 0 and 40
    self.cellToLoc = dict()
    self.trafficNet = trafficNet
    self.droneDenObsError = droneDenObsError
    # store data!
    self.storePropEnsembles = list()
    self.storeAhat = list()
    self.storeA = list()
    self.storeKalman = list()
    self.storeD = list()
    self.storeDmA = list()
    self.storeInvPart = list()
    self.storeCovPart = list()
    self.storeAhatPrime = list()

  def createLocToCell(self):
    '''
    creates the mapping between (linkid, cell) to
    global cellindex between (0,40) across all links
    '''
    cellindex = 0
    for linkID in self.trafficNet.linkDict:
      if linkID is not 9:
        for cellkey, cell in enumerate(self.trafficNet.linkDict[linkID].cells):
          self.locToCell[(linkID, cellkey)] = cellindex
          self.cellToLoc[cellindex] = (linkID, cellkey)
          cellindex += 1
    return None
  
  def genModErrorMatrix(self):
    '''
    generates a matrix of perturbations
    that will be used to perturb CTM forecasts
    '''
    self.modelErrorMatrix = np.random.normal(loc=0.0, scale=self.modelError, size=(self.stateDim, self.sampleSize))  # for general multivariate normal, this could have been generated using  numpy.random.multivariate_normal(mean, cov)
    return None
  
  def genObsErrorMatrix(self):
    '''
    generate a matrix of perturbations
    that will be used to perturb sensor
    observations
    Adjusts for randomness based on 
    congestion state
    note for future only considering drone obs in embedded EnKFs
    '''
    if (self.EnKFtype is 'CTM') and (self.droneLoc is not None):
      droneCell = self.locToCell[self.droneLoc]
      self.obsErrorMatrix = np.random.normal(loc=0.0, scale=self.obsError, size=(self.obsDim, self.sampleSize))  # for general multivariate normal, this could have been generated using  numpy.random.multivariate_normal(mean, cov)
      self.obsErrorMatrix[droneCell] = np.random.normal(loc=0.0, scale=self.droneDenObsError, size=self.sampleSize)  # lower error at location of  drone!
    else:
      self.obsErrorMatrix = np.random.normal(loc=0.0, scale=self.obsError, size=(self.obsDim, self.sampleSize))
    return None
      
  def getUpdatedEnsembles(self):
    '''
    get the updated ensembles in list of lists format
    needed for CTM processing, each sublist is a list
    of densities
    '''
    Atranspose = np.transpose(self.A)
    listofLists = Atranspose.tolist()
    return listofLists
  
  def getMean(self):
    '''
    adds some layer of protection
    '''
    return self.mean
  
  def getP(self):
    '''
    return P, scales by 1/(N-1) as needed
    in rest of code here this scale is 
    ignored since it cancels out
    keeps self.P intact
    '''
    P = (1.0/(self.sampleSize-1)) * self.P
    return P
    
  def getPostDist(self):
    '''
    updates mean and covariance matrix based on
    observations
    '''
    if self.nonLinearObs is False:
      self.A = self.A + np.dot(self.K, self.D - np.dot(self.H, self.A))
      scaleMatrix = np.full((self.sampleSize, self.sampleSize), 1.0/self.sampleSize)
      self.Abar = np.dot(self.A, scaleMatrix)
      self.mean = self.Abar[:,0]
      self.P = self.P - np.dot(self.K, np.dot(self.H, self.P))
      
    elif self.nonLinearObs is True:
      self.A = self.A + np.dot(self.K, self.D - self.Ahat)
      self.storeDmA.append(self.D - self.Ahat)
      scaleMatrix = np.full((self.sampleSize, self.sampleSize), 1.0/self.sampleSize)
      self.Abar = np.dot(self.A, scaleMatrix)
      self.mean = self.Abar[:,0]
      self.Aprime = self.A - self.Abar
      self.P = np.dot(self.Aprime, np.transpose(self.Aprime))
      self.storeA.append(self.A)
    return self.mean, self.P
    
  def getKalmanGain(self):
    '''
    computes the Kalman gain
    '''
    if self.nonLinearObs is False:
      temp1 = np.dot(self.P, np.transpose(self.H))
      temp2 = np.dot(self.H, self.P)
      temp2 = np.dot(temp2, np.transpose(self.H))
      temp2 = temp2 + self.R
      temp2 = np.linalg.inv(temp2)
      self.K = np.dot(temp1, temp2)
    elif self.nonLinearObs is True:
      Ahat0 = self.m(self.A[0],self.assimDen[0])  # Warning, works specifically with problem at hand
      Ahat1 = self.m(self.A[1], self.assimDen[1])
      self.Ahat = np.stack((Ahat0,Ahat1)) # compute Ahat through the m function
      self.storeAhat.append(self.Ahat)
      scaleMatrix = np.full((self.sampleSize, self.sampleSize), 1.0/self.sampleSize)
      self.Ahatbar = np.dot(self.Ahat, scaleMatrix)
      self.AhatPrime = self.Ahat - self.Ahatbar
      self.storeAhatPrime.append(self.AhatPrime)
      temp1 = np.dot(self.Aprime, np.transpose(self.AhatPrime))
      self.storeCovPart.append(temp1)
      temp2 = np.dot(self.AhatPrime, np.transpose(self.AhatPrime))
      temp2 = temp2 + self.R
      temp2 = np.linalg.inv(temp2)
      self.storeInvPart.append(temp2)
      self.K = np.dot(temp1, temp2)
      self.storeKalman.append(self.K)
    return None
  
  def getPriorDist(self):
    '''
    generate prior t+1|t
    updates mean and covariance based on
    predictions
    '''
    scaleMatrix = np.full((self.sampleSize, self.sampleSize), 1.0/self.sampleSize)
    self.Abar = np.dot(self.A, scaleMatrix)
    self.mean = self.Abar[:,0]
    self.Aprime = self.A - self.Abar
    self.P = np.dot(self.Aprime, np.transpose(self.Aprime))
    # self.P = (1.0/(self.sampleSize-1)) * self.P
    return self.mean, self.P
  
  def addModelNoise(self, forecasts):
    '''
    forecasts is a list of lists, where sublists
    are model updates for different ensemble
    members
    --------
    NOTE:
    forecasts do not include noise, this method
    adds the noise!
    --------
    returns noisy forecasts
    '''
    self.A = np.array(forecasts)
    self.A = np.transpose(self.A)
    self.genModErrorMatrix()
    self.A = self.A + self.modelErrorMatrix
    self.storePropEnsembles.append(self.A)
    return None

  def addObsNoise(self, observations):
    '''
    observations is a single list of
    sensor measurements
    adds obs noise to observations
    returns noisy observations
    if model has different congestion states
    adjusts observation noise accordingly
    in genObsErrorMatrix
    '''
    tempList = list()
    for sample in range(self.sampleSize):
      tempList.append(observations)
    self.D = np.array(tempList)
    self.D = np.transpose(self.D)
    self.genObsErrorMatrix()
    self.D = self.D + self.obsErrorMatrix
    self.storeD.append(self.D)
    return None
  
  def getObsCov(self):
    '''
    get the observation covariance matrix
    '''
    self.R = np.dot(self.obsErrorMatrix, np.transpose(self.obsErrorMatrix))
    # self.R = (1.0/(self.sampleSize-1)) * self.R
    return None
  
  def EnKFStep(self, forecasts, observations):
    '''
    implements one EnKF forecast and
    assimilation step and updates
    mean and covariance (get posterior)
    then samples and returns next step
    assimilated states
    '''
    self.createLocToCell()
    self.addModelNoise(forecasts)
    self.addObsNoise(observations)
    self.getObsCov()
    self.getPriorDist()
    self.getKalmanGain()
    self.getPostDist()
    return self.getUpdatedEnsembles()









    
    
  
