# -*- coding: utf-8 -*-
"""
abstract link class
includes methods for updating parameters using parameter EnKF estimates

@author: cesny
"""


# link parameters, here common for all links, can be different, read in readLinks
critDen = 100.0 # (veh/km)
linkType = 'CTM'  # specifies the link type
ffs = 100.0  # free flow speed in km/hr
jamDen = 300.0  # veh/km
qcap = ffs * critDen  # veh/hr  (assuming triangular FD)
bws = (ffs * critDen) / (jamDen - critDen)  # backward wave speed  (km/hr)
length = 1.5  # km
timeStep = 10  # seconds  (needed to define num cells in case of CTM, delx=ufdelt)
params = {'critDen':critDen, 'ffs':ffs, 'jamDen': jamDen, 'qcap': qcap, 
          'bws':bws, 'length':length, 'timeStep': timeStep, 'linkType': 'CTM'}


class Link:
  """
  this class is for link objects
  upstreamPathCount is a dict of dicts, time: subdict
  subdict is a dict of paths and their flow values
  """
  def __init__(self, linkID, unode, dnode, params):
    self.ID = linkID
    self.unode = unode  # upstream node object
    self.dnode = dnode
    self.params = params
    self.params['qcap'] = params['ffs'] * params['critDen']  # cap in veh/hr
    self.params['bws'] = (self.params['ffs'] * self.params['critDen']) / (self.params['jamDen'] - self.params['critDen'])  # backward wave speed in km/hr
    self._upstreamCounts = dict()  # records cumulative counts across time steps
    self._downstreamCounts = dict()
    self.inFlow = 0
    self.outFlow = 0  # current inFlow and outFlow into the link
    
  def calculateSendingFlow(self, time, timeStep):
    """
    overwrite in subclass
    """
    pass
  
  def calculateReceivingFlow(self, time, timeStep):
    """
    overwrite in subclass
    """
    pass
  
  def linkUpdate(self, time):
    """
    performs any internal calculations, puts flows on links, and 
    removes flows from downstream end of link
    """
    self.flowIn(time)
    self.flowOut(time)
    return None
  
  def upstreamCount(self, time):
    """
    return the cumulative entries to a link up to time t
    """
    if time < 0:
      return 0
    return self._upstreamCounts(time)
  
  def downstreamCount(self, time):
    """
    returns the cumulative exists from a link
    """
    if time < 0:
      return 0
    return self._downstreamCounts(time)
  
  def vehiclesOnLink(self, time):
    """
    returns vehicles on link
    """
    return self.upstreamCount(time) - self.downstreamCount(time)
  
  def flowIn(self, time):
    """
    adds flow to link based on inFlows from 
    transitionFlows
    """
    self._upstreamCounts[time] = self.inFlow  # I believe that this should be +=self.inflow but for our purposes it does not matter
    return None
  
  def flowOut(self, time):
    """
    removes flow from the downstream end of the link
    based on outFlow from transitionFlows
    """
    self._downstreamCounts[time] = self.outFlow
    return None
  
  def linkDensity(self, time=None):
    """
    overwriten in cell transmission model to return
    density at every cell
    """
    return self.vehiclesOnLink(time) / self.params['length']  # density in veh/km
  
  def updateVmaxCritDen(self, newVmax, newCritDen):
    """
    update the maximum speed
    note assuming that the backward wave speed is fixed
    """
    if newVmax > 110:
      print('.. WARNING! CFL condition violated ...')
    self.params['ffs'] = newVmax
    self.params['critDen'] = newCritDen
    self.params['qcap'] = newVmax * newCritDen
    # self.params['bws'] = (newVmax * self.params['critDen']) / (self.params['jamDen'] - self.params['critDen'])  # remains fixed across iterations
    return None
  
      
      
      
      
      
      
      
      
      
      
      
      
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
    
    
    
    
    
    





