# -*- coding: utf-8 -*-
"""
this contains utilities

@author: cesny
"""
from network import Network
import numpy as np
import copy as cp


def readData(textfile):
  """
  reads the text file
  returns two lists with density and velocity
  measured at a specific point
  """
  try:
    with open(textfile) as tf:
      density = dict()
      speed = dict()
      for line in tf:
        data = line.strip().split(';')
        first_number = 0
        try:
          first_number = float(data[0])
        except:
          pass
        if first_number == 1:
          roadid_str_list = data[2].split('-')
          roadid = (float(roadid_str_list[0]), float(roadid_str_list[1]), \
                      float(roadid_str_list[2]))
          time = data[1].split('-')
          timeStep = float(time[0]) / 10
          density.setdefault(timeStep,[])
          speed.setdefault(timeStep,[])
          thresholds = [120.0, 370.0, 620.0, 870.0, 1120.0, 1370.0]  # thresholds are used so that you take a measurement in each cell
          if (roadid[0] != 9) and (roadid[1] in thresholds):
            density[timeStep].append(float(data[3]))
            if (roadid[0] == 2 and roadid[1] == 120.0) or (roadid[0] == 7 and roadid[1] == 120.0):  # storing first cell only
              if (float(data[3]) != 0.0):  #density is not zero
                speed[timeStep].append(float(data[5]))
              else:
                speed[timeStep].append(100.0)
  except IOError as ioerr:
    print('failed to read' + str(ioerr))
  return density, speed


def setCTMVehicles(trafficNet, EnKFensemble):
  '''
  sets the vehicles in CTM traffic model
  based on one particular ensemble output
  an ensemble is a list of 40 CTM densities
  across cells, set ramp cells to zero to
  allow vehicles to exit
  '''
  cellindex = 0
  for linkID in trafficNet.linkDict:
    if linkID is not 9:
      for cell in trafficNet.linkDict[linkID].cells:
        EnKFveh = EnKFensemble[cellindex] * cell.length  # convert density to vehicles
        cell.vehicles = EnKFveh  # assign EnKF determined vehicles to cell
        cellindex += 1
    if linkID is 9:
      for cell in trafficNet.linkDict[linkID].cells:
        cell.vehicles = 0
  return trafficNet


def forwardCTMPropagation(time, trafficNet, EnKFensembles):
  '''
  propagates a set of EnKF ensembles forward
  each EnKF ensemble is a list of 40 density values across
  cells
  flow is propagated using cell transmission model
  return a list of lists with propagated ensembles
  '''
  propagatedEnsembles = list()
  for ensemble in EnKFensembles:
    trafficNet = setCTMVehicles(trafficNet, ensemble)  # set current ensembles
    propagatedEnsembles.append(trafficNet.loadNetworkStep(time)[0])  # propagate the ensembles forward using model
  return propagatedEnsembles


def CTMcreateInitialEnsemble(stateDim, ensembleSize, modSTDV, bestguess=20):
  '''
  creates an initial ensemble around a best guess
  estimate of the state
  best guess is a single number for best guess average
  densities on the states
  '''
  return np.random.normal(loc=bestguess, scale=modSTDV, size=(ensembleSize,stateDim)).tolist()

  
def VmaxCreateInitialEnsemble(VstateDim,Vensembles,VmodSTDV, bestguess=80):
  '''
  creates an initial ensemble for vmax
  '''
  return np.random.normal(loc=bestguess, scale=VmodSTDV, size=(Vensembles,VstateDim)).tolist()

  
def m(vmax, rho):
  '''
  this function is used to get the model predicted
  velocity measurement from vmax (i.e. model predicted diagnostic variable
  for nonlinear EnKF observation matrix)
  original back wave based on vmax=100, rhocr=80, jamDen = 300 - can be
  fitted by measuring traffic considtion under normal conditions
  '''
  v=np.full(vmax.shape,0.0)
  for key, vmaxVal in enumerate(vmax):  
    rhoCritical = (80.0*100*300)/(vmaxVal*(300 - 80) + 80*100)  # update rho critical given current vmax
    if rho < rhoCritical:
      v[key] = vmaxVal
    elif rho > rhoCritical:
      v[key] = float(vmaxVal*(rhoCritical)*(300.0 - rho))/(rho*(300.0 - rhoCritical))
  return v  # generate the predicted observation, this will be used to update vmax


def VmaxtoCritDen(vmax):
  '''
  defines the relationship based on maintaining uncongested
  backwave
  '''
  rhoCrit=np.full(vmax.shape,0.0)
  for key, vmaxVal in enumerate(vmax):  
    rhoCritical = (80.0*100*300)/(vmaxVal*(300 - 80) + 80*100)
    rhoCrit[key] = rhoCritical
  return rhoCrit


def createLocToCell(trafficNet):
  '''
  creates the mapping between (linkid, cell) to
  global cellindex between (0,40) across all links
  '''
  cellindex = 0
  locToCell = dict()
  cellToLoc = dict()
  for linkID in trafficNet.linkDict:
    if linkID is not 9:
      for cellkey, cell in enumerate(trafficNet.linkDict[linkID].cells):
        locToCell[(linkID, cellkey)] = cellindex
        cellToLoc[cellindex] = (linkID, cellkey)
        cellindex += 1
  return locToCell


def cellToLength(listofLocations, trafficNet):
  '''
  returns UAV location on the road network in km
  across time, input list of UAV location (linkID, linkCell)
  '''
  DronePositions = list()
  LocToCell = createLocToCell(trafficNet)
  for location in listofLocations:
    cellLocation = LocToCell[location]
    physicalLocation = (5.0/18)*cellLocation + (5.0/36)
    DronePositions.append(physicalLocation)
  return DronePositions


def lengthToCell(locKm):
  '''
  from position in km to cell number
  '''
  cell_loc = list()
  for loc in locKm:
     cell_loc.append(int(loc*(18.0/5)-0.5))
  return cell_loc