# -*- coding: utf-8 -*-
"""
abstract node class

@author: cesny
"""

class Node:
  """
  this class is for Node objects
  """
  def __init__(self, nodeID, nodeModel, fstar, rstar):
    self.ID = nodeID
    self.model = nodeModel
    self.downstreamLinks = list()
    self.upstreamLinks = list()
    self.fstar = list()
    self.rstar = list()
    self._processStars(fstar, rstar)
    
  def _processStars(self, fstar, rstar):
    """
    creates fstar rstar from input files
    """
    for star in fstar:
      intStar = int(star)
      self.fstar.append(intStar)
      
    for star in rstar:
      intStar = int(star)
      self.rstar.append(intStar)
    return None
    
  def nodeUpdate(self, time, timeStep):
    """
    moves flow by calculating sending/receiving flows and
    calling the calculateTransitionFlows to get transitionFlow dict
    this method also updates link counts
    """
    sendingFlow = dict()
    receivingFlow = dict()
    for inLink in self.upstreamLinks:
      sendingFlow[inLink.ID] = inLink.calculateSendingFlow(time, timeStep)
    for outLink in self.downstreamLinks:
      receivingFlow[outLink.ID] = outLink.calculateReceivingFlow(time, timeStep)
    # add transitiionFlows as a node attribute
    self.transitionFlows = self.calculateTransitionFlows(sendingFlow, receivingFlow)
    linkInflows = dict()
    linkOutflows = dict()
    for inLink in self.transitionFlows:
      linkOutflows[inLink] = sum(self.transitionFlows[inLink].values())
      for outLink in self.transitionFlows[inLink]:
        linkInflows[outLink] = linkInflows.setdefault(outLink, 0.0) + self.transitionFlows[inLink][outLink] 
    for inLink in self.upstreamLinks:
      inLink.outFlow = linkOutflows[inLink.ID]
    for outLink in self.downstreamLinks:
      outLink.inFlow = linkInflows[outLink.ID]
    # now make sure you use those inflows and outflows to updateLinks for time+1
    return None
    
  def calculateTransitionFlows(self, sendingFlow, receivingFlow, proportions=None):
    """
    returns transition flow, which is a dict of dicts, keys are pair of inc
    out links and vals are num of vehicles that can move during time interval
    {a:{b:4, c:6}} a, b, and c are links, 4 & 6 are transitionFlows
    """
    pass
  
