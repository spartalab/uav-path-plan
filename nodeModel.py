# -*- coding: utf-8 -*-
"""
node models

@author: cesny
"""

from node import Node

class Zone(Node):
  def __init__(self, nodeID, nodeModel, fstar, rstar):
    Node.__init__(self, nodeID, nodeModel, fstar, rstar)
    self.__initialize()
    
  def __initialize(self):
    """
    sets subtype of node and creates
    demandRates if origin
    """
    if len(self.fstar) == 0:
      # Node is destination
      self.subType = 'Destination'
    if len(self.rstar) == 0:
      self.subType = 'Origin'
      self.demandRates = dict()  # create a demand rates dict if it's an origin!
    if (len(self.rstar) != 0) and (len(self.fstar) != 0):
      raise Exception('... wrong zone initialization ...')
    if (len(self.rstar) >= 1) and (len(self.fstar) >= 1):
      raise Exception('... code note ready for multiple ins or outs for des or orig ...') 
    return None
  
  def calculateTransitionFlows(self, sendingFlow, receivingFlow, proportions=None):
    """
    raises exception
    """
    raise Exception('... load vehicles for Zones error in calculate transition flows ...')
    return None
  
  def nodeUpdate(self, time, timeStep):
    """
    override the nodeUpdate method to update inFlows and outFlows
    based on loading vehicles or removing them
    """
    if self.subType == 'Origin':
      for outLink in self.downstreamLinks:
        outLink.inFlow = self.demandRates[time] * (1.0/3600) * timeStep  # demand is in veh/hr, have to convert to timestep
    if self.subType == 'Destination':
      for inLink in self.upstreamLinks:  # if it's the destination, all the link from upstream gets out
        inLink.outFlow = inLink.calculateSendingFlow(time, timeStep)
    return None
  
    


class SeriesNode(Node):
  def __init__(self, nodeID, nodeModel, fstar, rstar):
    Node.__init__(self, nodeID, nodeModel, fstar, rstar)
  
  def calculateTransitionFlows(self, sendingFlow, receivingFlow, proportions=None):
    """
    sendingFlow and receivingFlow are dictionaries
    sendingFlow = {uplink1: val, uplink2: val..}
    """
    transitionFlows = dict()
    for inLinkID in self.rstar:
      transitionFlows[inLinkID] = dict()
    
    upLink = self.rstar[0]
    downLink = self.fstar[0]
    transitionFlows[upLink][downLink] = min(sendingFlow[upLink], receivingFlow[downLink])
    return transitionFlows
  

class DivergeNode(Node):
  def __init__(self, nodeID, nodeModel, fstar, rstar):
    self.proportions = dict()  # a dictionary with props of flow to links, considering that proportions are fixed
    Node.__init__(self, nodeID, nodeModel, fstar, rstar)
  
  def _processStars(self, fstar, rstar):
    """
    overrides node processStars to create proportions if they are static
    """
    for star in rstar:
      intStar = int(star)
      self.rstar.append(intStar)
    self.proportions[self.rstar[0]] = dict()
    for star in fstar:
      link, prop = star.split(':')
      intLink = int(link)
      floatProp = float(prop)
      self.fstar.append(intLink)
      self.proportions[self.rstar[0]][intLink] = floatProp    
    return None
  
  def calculateTransitionFlows(self, sendingFlow, receivingFlow, proportions=None):
    """
    sendingFlow and receivingFlow are dictionaries
    sendingFlow = {uplink1: val, uplink2: val..}
    """
    transitionFlows = dict()
    for inLinkID in self.rstar:
      transitionFlows[inLinkID] = dict()
    inLink = self.rstar[0]
    thetas = list()
    for outLink in self.fstar:
      if (sendingFlow[inLink] != 0) and (self.proportions[inLink][outLink] != 0):
        thetas.append(receivingFlow[outLink] / ((self.proportions[inLink][outLink])*(sendingFlow[inLink])))
    thetas.append(1.0)
    theta = min(thetas)
    for outLink in self.fstar:
      transitionFlows[inLink][outLink] = theta * self.proportions[inLink][outLink] * sendingFlow[inLink]
    return transitionFlows

