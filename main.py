
"""
primary UAV navigation and estimation framework

@author: cesny
"""

from network import Network
import numpy as np
from EnKF import EnKF
import copy as cp
import matplotlib.pyplot as plt
from findPath import findPath
from utils import readData, setCTMVehicles, forwardCTMPropagation, CTMcreateInitialEnsemble, VmaxCreateInitialEnsemble, m, VmaxtoCritDen, createLocToCell, cellToLength, lengthToCell

 

if __name__ == '__main__':
  # create traffic network
  simTime = 4490
  simTimeStep = 10
  totalTimeSteps = range(int(np.ceil(float(simTime)/simTimeStep)) + 1)
  linkfile = 'VISSIMnetwork/links.txt'
  nodefile = 'VISSIMnetwork/nodes.txt'
  demandfile = 'VISSIMnetwork/demand6600.txt'
  trafficNet = Network(simTime, simTimeStep, nodefile, linkfile, demandfile)
  LocToCell = createLocToCell(trafficNet)
  # laod data observations from VISSIM
  denData, spData = readData('data/model_001_Link Segment Results-6600.att')
  
  # true incident Vf
  trueVf=20.0
  
  # UAV path planning weight lambda
  pathWeight=1.0
  
  # set initial UAV location, link 5 cell 0
  droneLocation = (5,0)
  
  # specify parameters for CTM-EnKF
  totalcells = 0
  for linkID in trafficNet.linkDict:
    if linkID is not 9:
      totalcells += trafficNet.linkDict[linkID].numCells
  HCTM = np.array(np.identity(totalcells).tolist())
  CTMobsSTDV = 10  # standard deviation veh/km
  CTMdrObsSTDV = 2 # standard deviaiton veh/km of drone observations
  CTMmodSTDV = 5  # standard deviation veh/km
  CTMstateDim = 40  # 40 cells with monitored densities
  CTMobsDim = 40  # 40 cells with monitored densities
  CTMensembles = 100  # number of ensembles in EnKF
  EnKFCTM = EnKF(obsError=CTMobsSTDV, modelError=CTMmodSTDV, sampleSize=CTMensembles, stateDim=CTMstateDim, obsDim=CTMobsDim, H=HCTM, droneLoc=droneLocation, trafficNet=trafficNet, droneDenObsError=CTMdrObsSTDV)
  
  # specify parameters for velocity EnKF when velocities are observed
  VobsSTDV = 5 # obs standard deviation!
  VmodSTDV = 5  # model random walk standard dev. in km/hr
  VstateDim = 2  # two incident prone regions
  VobsDimV = 2  # observing velocities on those two regions
  Vensembles = 100  # number of ensembles in EnKF
  EnKFV = EnKF(obsError=VobsSTDV, modelError=VmodSTDV, sampleSize=Vensembles, stateDim=VstateDim, obsDim=VobsDimV, m=m, assimilatedDensities=[0,0], EnKFtype='Vmax', nonLinearObs=True, droneLoc=droneLocation, trafficNet=trafficNet)
  
  # specify parameters for velocity EnKF when we obtain direct uf observations
  VdrObsSTDV = 10  # error of observing the true uf value
  VobsDimVf = 1  # when the drone observes it only does so at one location
  
  # creates initial ensembles
  CTMensembles = CTMcreateInitialEnsemble(CTMstateDim, CTMensembles, CTMmodSTDV)
  VmaxEnsembles = VmaxCreateInitialEnsemble(VstateDim,Vensembles,VmodSTDV)
  
  # store data
  firstIncidentDen = list()
  secIncidentDen = list()
  Vmax1Estimated = list()
  Vmax2Estimated = list()
  critDenEstimated = list()
  listofTime = list()
  VmaxlistofTime = list()
  storeDenTotal=list()
  objective = list()
  velObj = list()
  storeDenInc = list()
  storeDroneLocation = list()
  droneLocCell=list()
  droneLocKm=list()
  
  # simulate
  for time in totalTimeSteps:  # cell indices 6 and 32 for inc1 and inc2, respectively (i.e., those are the incident prone locations)
    CTMensembles = forwardCTMPropagation(time, trafficNet, CTMensembles)  # propagate ensembles using CTM
    CTMensembles = EnKFCTM.EnKFStep(CTMensembles, denData[time])  # data assimilation, get updated density ensembles from EnKF
    firstIncidentDen.append(EnKFCTM.mean[6])  # add best estimate of den in incident location to list
    secIncidentDen.append(EnKFCTM.mean[32])
    storeDenTotal.append(EnKFCTM.mean)
    listofTime.append(time*10/60.0)
    if (time % 30 == 0):  # if you observe a velocity measurement
      print('velocity observation')
      VmaxlistofTime.append(time)
      EnKFV.nonLinearObs = True  # nonlinear relationship when velocity is observed
      EnKFV.H = None  # there is no H matrix
      # get assimilated densities
      assimDensities = [EnKFCTM.mean[6], EnKFCTM.mean[32]]  # current density, use in nonlinear observations to determine vmax
      EnKFV.assimDen = assimDensities
      storeDenInc.append(assimDensities)  # stores the denisities at incident-prone locations when velocity measurements are collected
      # adjust observations
      EnKFV.obsError = VobsSTDV
      EnKFV.obsDim = VobsDimV
      VmaxEnsembles = EnKFV.EnKFStep(VmaxEnsembles, spData[time])  # propagate Vmax ensembles using random walk and assimilate observed data
      # store results
      Vmax1Estimated.append(EnKFV.mean[0])
      Vmax2Estimated.append(EnKFV.mean[1])
      critDenEstimated.append(VmaxtoCritDen(EnKFV.mean))
      # update traffic parameters
      trafficNet.updateVmaxCritDen(EnKFV.mean, VmaxtoCritDen(EnKFV.mean))  # Now the parameters are updates for the links of interest at the locations of interest
      # store objective
      Vobj = np.trace(EnKFV.getP())
      print('post velocity trace: ', EnKFV.getP())
      velObj.append(Vobj)
      
    if (droneLocation[0] == 2) or (droneLocation[0] == 7):  # if UAV at incident location
      if time in VmaxlistofTime:
        print("UAV at incident location when velocity is measured")
      VmaxlistofTime.append(time)
      EnKFV.nonLinearObs = False  # linear relationship when drone at incident location (direct uf observation)
      if droneLocation[0] == 2:
        EnKFV.H = np.array([[1.0, 0.0]])
      if droneLocation[0] == 7:
        EnKFV.H = np.array([[0.0, 1.0]])
      # adjust observations
      EnKFV.obsError = VdrObsSTDV
      EnKFV.obsDim = VobsDimVf
      VfObserved = [trueVf]
      VmaxEnsembles = EnKFV.EnKFStep(VmaxEnsembles, VfObserved)  # the observed uf is only the one at the incident location, propagate Vmax ensembles using random walk and assimilate observed data
      # store results
      Vmax1Estimated.append(EnKFV.mean[0])
      Vmax2Estimated.append(EnKFV.mean[1])
      critDenEstimated.append(VmaxtoCritDen(EnKFV.mean))
      # update traffic parameters
      trafficNet.updateVmaxCritDen(EnKFV.mean, VmaxtoCritDen(EnKFV.mean))  # Now the parameters are updates for the links of interest at the locations of interest
      # store objective
      Vobj = np.trace(EnKFV.getP())
      print('drone uf obs. at: ', droneLocation, EnKFV.getP())
      print('updated ensembles: ', droneLocation, np.transpose(np.array(VmaxEnsembles))[:,0:3])
      print(' ')
      velObj.append(Vobj)  # store the variation in the trace of the EnKF-V (uf)
    
    # UAV path planning, note the objective below is NOT along candidate paths! It is the current instantaneous objective!
    obj= (pathWeight * np.trace(EnKFV.getP()) / 2.0) + ((1-pathWeight) * np.trace(EnKFCTM.getP()) / 40.0)  # path planning objective
    objective.append(obj)
    # update the UAV location and update filters
    print('pre-find path ensembles: ', np.transpose(np.array(VmaxEnsembles))[:,0:3])  # sanity check
    explorePath = findPath(location=droneLocation, time=time, trafficNet=cp.deepcopy(trafficNet), EnKFCTM=cp.deepcopy(EnKFCTM), EnKFV=cp.deepcopy(EnKFV), weight=pathWeight)
    droneLocation = explorePath.updateLocation()
    print('post-find path ensembles: ', np.transpose(np.array(VmaxEnsembles))[:,0:3])  # sanity check
    print('post-find path ensembles from EnKF: ', np.transpose(np.array(EnKFV.getUpdatedEnsembles()))[:,0:3])  # sanity check
    EnKFCTM.droneLoc = droneLocation  # update UAV loc. in CTM EnKF
    EnKFV.droneLoc = droneLocation  # update UAV loc. in uf EnKF
    print(' ')
    print(' ... ')
    print('drone currently at: ', droneLocation)
    storeDroneLocation.append(droneLocation)
    droneLocCell.append(LocToCell[droneLocation])

  # determine position of UAV in km from start of road
  droneLocKm.append(cellToLength(storeDroneLocation, trafficNet))
  
  
  ############# data
  # create list of observations at critical time steps for sanity checks
  spdict = dict()
  for time in totalTimeSteps:
    if time % 30 == 0:
        spdict[time]=spData[time]
  
  ############# plotting
  VmaxTrue = list()
  for time in VmaxlistofTime:
    VmaxTrue.append(20)
  
  