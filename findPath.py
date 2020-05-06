#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
this module is for implementing one step lookahead control 

determines drone location at next step based on information maximizing
control algorithm

@author: cesny
"""
import numpy as np
from utils import setCTMVehicles, forwardCTMPropagation, CTMcreateInitialEnsemble, VmaxCreateInitialEnsemble, m, VmaxtoCritDen, cellToLength, lengthToCell




class findPath:
  '''
  this class is for determining next drone 
  location based on A-optimal control
  '''
  def __init__(self, location, time, trafficNet, EnKFCTM, EnKFV, timeHorizon=None, weight=0.5):
    self.location = location  # current drone location, defined as a tuple (linkID, cell), cell count from zero
    self.time = time  # current time
    self.timeHorizon = timeHorizon  # time horizon to do MPC (number of timeSteps), set dynamically till drone visits all cells in each path, can assign otherwise
    self.trafficNet = trafficNet  # the traffic network class
    self.EnKFCTM = EnKFCTM  # the ensemble kalman filtering class for densities
    self.EnKFV = EnKFV  # the ensemble kalman filtering class, use to predict covariance matrix
    self.cellToLoc = dict()
    self.locToCell = dict()  # maps the tuple (link, cell) to cell between 0 and 40
    self.weight = weight  # this is the weight of vmax vs densities trace, weight corresponds to vmax, (1-w) corresponds to weight of densities trace
  
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
  
  def generateDronePaths(self):
    '''
    gets the possible paths the drone can take based
    on its current location. A path is a collection of
    cells. Returns self.paths a dict of dicts determining the
    cellid at every time step for possible paths
    '''
    self.dronePaths = dict()
    self.dronePaths['left'] = dict()
    self.dronePaths['right'] = dict()
    droneCell = self.locToCell[self.location]
    leftPath = list(range(0,droneCell+1))
    leftPath.reverse()
    rightPath = list(range(droneCell, 40))
    # assuming at every time step the drone can move one cell, determine its path across time
    for key, lcell in enumerate(leftPath):
      if self.time+key <= 449:  # limited by time horizon
        self.dronePaths['left'][self.time+key] = lcell
    for key, rcell in enumerate(rightPath):
      if self.time+key <= 449:
        self.dronePaths['right'][self.time+key] = rcell
    return None
  
  def getObservations(self):
    '''
    use traffic network to simulate paths based on current
    condition and return expected observations for the given
    paths (left path and right path). Observations is a list over
    the 40 cells.
    '''
    self.pathObservations = dict()  # a dictionary with mean traffic state at every time step from now as predicted by propagated CTM ensembles
    self.pathObservations['left'] = dict()
    self.pathObservations['right'] = dict()
    loadRange = max(len(self.dronePaths['left']), len(self.dronePaths['right']))
    storeResults = dict()
    CTMensembles = self.EnKFCTM.getUpdatedEnsembles()  # current ensembles
    for lr in range(loadRange):
      CTMensembles = forwardCTMPropagation(self.time + lr, self.trafficNet, CTMensembles)
      storeResults[self.time + lr] =  [float(sum(col))/len(col) for col in zip(*CTMensembles)]  # store average of propagated ensembles as expected observed true state
    
    for time in self.dronePaths['left']:
      self.pathObservations['left'][time] = storeResults[time]
    for time in self.dronePaths['right']:
      self.pathObservations['right'][time] = storeResults[time]
    return None
  
  def getCovarianceMatrices(self):
    '''
    use the "observations" (from propagated ensembles) to determine
    the final covariance matrix on the densities and vmax
    covariance on vmax set manually based on actual conditions, covariance
    of densities computed from EnKF with CTM forward simulation results as 
    expected observations, drone observations accounted for by lower observation
    error
    '''
    self.finalCovariancesCTM = dict()
    self.finalCovariancesVmax = dict()
    # densities covariance matrix
    CTMensemblesLeft = self.EnKFCTM.getUpdatedEnsembles()
    CTMensemblesRight = self.EnKFCTM.getUpdatedEnsembles()
    VmaxEnsemblesLeft = self.EnKFV.getUpdatedEnsembles()
    VmaxEnsemblesRight = self.EnKFV.getUpdatedEnsembles()
    for time in self.dronePaths['left']:
      CTMensemblesLeft = forwardCTMPropagation(time, self.trafficNet, CTMensemblesLeft)
      self.EnKFCTM.droneLoc = self.cellToLoc[self.dronePaths['left'][time]]  # update drone location according to base policy, used for precise observations
      CTMensemblesLeft = self.EnKFCTM.EnKFStep(CTMensemblesLeft, self.pathObservations['left'][time])  # update the ensembles
    self.finalCovariancesCTM['left'] = self.EnKFCTM.getP()  
    
    for time in self.dronePaths['right']:
      CTMensemblesRight = forwardCTMPropagation(time, self.trafficNet, CTMensemblesRight)
      self.EnKFCTM.droneLoc = self.cellToLoc[self.dronePaths['right'][time]]  # update drone location according to base policy, used for precise observations
      CTMensemblesRight = self.EnKFCTM.EnKFStep(CTMensemblesRight, self.pathObservations['right'][time])
    self.finalCovariancesCTM['right'] = self.EnKFCTM.getP()
    
    # update the uf estimates
    self.EnKFV.obsError = 10   
    self.EnKFV.obsDim = 1
    EnKFVmean = self.EnKFV.getMean()
    # VfObserved = [20.0]
    self.EnKFV.nonLinearObs = False
    # do left
    self.EnKFV.H = np.array([[1.0, 0.0]])
    print(' ')
    print('pre-update left: ', np.transpose(np.array(VmaxEnsemblesLeft))[:,0:3])
    VmaxEnsemblesLeft = self.EnKFV.EnKFStep(VmaxEnsemblesLeft, [EnKFVmean[0]])
    self.finalCovariancesVmax['left'] = self.EnKFV.getP()
    print('left path: ', self.finalCovariancesVmax['left'], np.transpose(np.array(VmaxEnsemblesLeft))[:,0:3])
    print(' ')
    # do right
    self.EnKFV.H = np.array([[0.0, 1.0]])
    print('pre-update right: ', np.transpose(np.array(VmaxEnsemblesRight))[:,0:3])
    VmaxEnsemblesRight = self.EnKFV.EnKFStep(VmaxEnsemblesRight, [EnKFVmean[1]])
    self.finalCovariancesVmax['right'] = self.EnKFV.getP()
    print('right path: ', self.finalCovariancesVmax['right'], np.transpose(np.array(VmaxEnsemblesRight))[:,0:3])
    print(' ')
    return None
    
  def getObjective(self):
    '''
    determines the objective function as a weighted form
    of the trace of the covariance matrices for densities and vmax
    corrects for dimension and scale differences
    returns objective of left path and objective of right path in a dict
    '''
    self.ObjectiveVal = dict()
    self.ObjectiveVal['left'] = (self.weight * np.trace(self.finalCovariancesVmax['left']) / 2.0) + ((1-self.weight) * np.trace(self.finalCovariancesCTM['left']) / 40.0)
    self.ObjectiveVal['right'] = (self.weight * np.trace(self.finalCovariancesVmax['right']) / 2.0) + ((1-self.weight) * np.trace(self.finalCovariancesCTM['right']) / 40.0)
    print(' ')
    print('objective left: ', np.trace(self.finalCovariancesVmax['left']) / 2.0, np.trace(self.finalCovariancesCTM['left']) / 40.0)
    print('objective right: ', np.trace(self.finalCovariancesVmax['right']) / 2.0, np.trace(self.finalCovariancesCTM['right']) / 40.0)
    return None
  
  def moveDrone(self, direction):
    '''
    changes the location of the drone
    '''
    if direction == 'left':
      currentCell = self.locToCell[self.location]
      if currentCell != 0:
        newLoc = currentCell - 1
        self.location =  self.cellToLoc[newLoc]
        
    elif direction == 'right':
      currentCell = self.locToCell[self.location]
      if currentCell != 39:
        newLoc = currentCell + 1
        self.location =  self.cellToLoc[newLoc]
    return self.location
  
  def updateLocation(self):
    '''
    determines the next step for the drone
    go left or go right, updates self.location
    '''
    self.createLocToCell()
    self.generateDronePaths()
    self.getObservations()
    self.getCovarianceMatrices()
    self.getObjective()
    minKey = min(self.ObjectiveVal, key=self.ObjectiveVal.get)
    return self.moveDrone(minKey)