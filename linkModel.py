# -*- coding: utf-8 -*-
"""
cell transmission model

@author: cesny
"""
import math
from link import Link

class Cell:
  def __init__(self, capacity, maxVehicles, delta, timeStep, ffs):
    self.capacity = capacity / 3600.0  # you have to divide by 3600 for veh/sec
    self.maxVehicles = maxVehicles  # maxVehicles per cell
    self.length = ffs * timeStep / 3600.0  # length in km
    self.delta = delta
    self.vehicles = 0
    self.timeStep = timeStep
    
  def calculateSendingFlow(self):
    return min(self.vehicles, self.capacity * self.timeStep)
  
  def calculateReceivingFlow(self):
    return min(self.delta * (self.maxVehicles - self.vehicles), self.capacity * self.timeStep)
  
  def removeVehicles(self, numVehicles):
    self.vehicles -= numVehicles
    return None
  
  def addVehicles(self, numVehicles):
    self.vehicles += numVehicles
    return None
  
  def cellDensity(self):
    return float(self.vehicles) /  self.length  # density in veh/km
    

class CTM(Link):
  def __init__(self, linkID, unode, dnode, params):
    Link.__init__(self, linkID, unode, dnode, params)
    # create a list of cells
    self.cells = list()
    #  freeflowtime is equiv to length/delx*delt
    #self.freeFlowTime = int((self.params['length']/(self.params['ffs'] / 3600.0) + self.params['timeStep'] - 1) / self.params['timeStep'])
    # timeStep is used as a parameter since it affects cell size
    self.numCells = math.ceil(self.params['length']/(self.params['ffs']*(1.0/3600)*self.params['timeStep']))
    for c in range(self.numCells):
      newCell = Cell(self.params['qcap'], self.params['jamDen'] * self.params['length'] / self.numCells,
                     self.params['bws'] / self.params['ffs'], self.params['timeStep'], self.params['ffs'])
      self.cells.append(newCell)
    
  def calculateSendingFlow(self, time, timeStep):
    """
    overwrites Link method
    """
    return self.cells[-1].calculateSendingFlow()
  
  def calculateReceivingFlow(self, time, timeStep):
    """
    overwrites Link method
    """
    return self.cells[0].calculateReceivingFlow()
  
  def linkUpdate(self, time):
    """
    performs any internal calculations, puts flows on links, and 
    removes flows from downstream end of link
    inFlow and outFlow were calculated based on the t-1
    sending and receiving flows
    """    
    cellTransitionFlow = list()
    # deal with intermediate cells
    for c in range(0, self.numCells-1):
      cellSendingFlow = self.cells[c].calculateSendingFlow()
      cellReceivingFlow = self.cells[c+1].calculateReceivingFlow()
      cellTransitionFlow.append(min(cellSendingFlow, cellReceivingFlow))
    for c in range(0, self.numCells-1):
      self.cells[c].removeVehicles(cellTransitionFlow[c])
      self.cells[c+1].addVehicles(cellTransitionFlow[c])
    self.flowIn(time)
    self.flowOut(time)
    return None
      
  def flowIn(self, time):
    Link.flowIn(self, time)
    self.cells[0].addVehicles(self.inFlow)
    return None
    
  def flowOut(self, time):
    Link.flowOut(self, time)
    self.cells[len(self.cells)-1].removeVehicles(self.outFlow)
    return None
  
  def linkDensity(self, time=None):
    """
    overwrites link method to return density at every cell
    """
    listofDensities = list()
    for cell in self.cells:
      listofDensities.append(cell.cellDensity())
    return listofDensities
  
  def updateVmaxCritDen(self, newVmax, newCritDen):
    """
    overwrites link method to update cells
    """
    Link.updateVmaxCritDen(self, newVmax, newCritDen)
    # now update cells, keep length and number of cells the same
    for cell in self.cells:
      cell.capacity = self.params['qcap'] / 3600.0 # already updated for link
      cell.delta = self.params['bws'] / self.params['ffs']  # already upated in link 
    return None

      