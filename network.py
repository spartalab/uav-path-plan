# -*- coding: utf-8 -*-
"""
implements network methods
including network loading

@author: cesny
"""

import nodeModel
import linkModel
import numpy as np

'''
simulation parameters used in VISSIM model
simTime = 3600
simTimeStep = 10
'''

def linkFactory(aClass, linkID, unode, dnode, params):
    """
    This function is a factory
    Classes are first class objects so they
    can be passed into this function
    :param aClass: A class object
    :param pargs: parameters
    :param kargs: key-value parameters
    :return: initiated type specific class
    """
    return aClass(linkID, unode, dnode, params)
  
def nodeFactory(aClass, nodeID, nodeModel, fstar, rstar):
    """
    This function is a factory
    Classes are first class objects so they
    can be passed into this function
    :param aClass: A class object
    :param pargs: parameters
    :param kargs: key-value parameters
    :return: initiated type specific class
    """
    return aClass(nodeID, nodeModel, fstar, rstar)


class Network:
  """
  This is a general network class for connecting links and nodes
  """
  def __init__(self, simTime, simStep, nodefile, linkfile, demandfile):
    self.simTime = simTime  # time horizon
    self.timeStep = simStep  # simulation time step
    self.totalTimesteps = range(int(np.ceil(float(self.simTime)/self.timeStep)) + 1)
    self.ODs = dict()  # create an OD dictionary, each OD is a class that stores demand
    self.nodeDict = dict()  # dictionary of nodes
    self.linkDict = dict()  # dictionary of links
    self._setupNetwork(nodefile, linkfile, demandfile)
  
  def _setupNetwork(self, nodefile, linkfile, demandfile):
    """
    sets up the network
    """
    self.readNodes(nodefile)
    self.readLinks(linkfile)
    self.readDemand(demandfile)
    return None
  
  def updateVmaxCritDen(self, newVmaxlist, newCritDenlist):
    """
    updates the critical densities in the network
    call before calling update Vmax
    """
    self.linkDict[2].updateVmaxCritDen(newVmaxlist[0], newCritDenlist[0])
    self.linkDict[7].updateVmaxCritDen(newVmaxlist[1], newCritDenlist[1])
    return None
  
  
  def readNodes(self, nfile):
    """
    reads node file, returns None
    """
    try:
      with open(nfile) as nf:
        print('... reading node file ...')
        nf.readline()
        for line in nf:
          data = line.strip().split('\t')
          if data[2] != '[]':
            fstar = data[2].strip('[]').split(',')
          else:
            fstar = []
          if data[3] != '[]':
            rstar = data[3].strip('[]').split(',')
          else:
            rstar = []
          self.nodeDict[int(data[0])] = nodeFactory(getattr(nodeModel, data[1]),\
                          int(data[0]), data[1], fstar, rstar)
          
    except IOError as ioerr:
      print('... error reading node file: ' + str(ioerr) + ' ...')
    return None
  
  def readLinks(self, lfile):
    """
    lfile: link text file
    network: network class
    params: dictionary with model parameters, may be read from text file
    reads links, returns None
    """
    if self.nodeDict == {}:
      raise Exception('... read nodes before links ...')
    
    try:
      with open(lfile) as lf:
        print('... reading link file ...')
        lf.readline()  # skip the first line
        for line in lf:
          data = line.strip().split('\t')
          params = dict()
          params['linkType'] = data[1]
          params['length'] = float(data[4])
          params['ffs'] = float(data[5])
          params['critDen'] = float(data[6])
          params['jamDen'] = float(data[7])
          params['timeStep'] = self.timeStep
          source = int(data[2])  # upstream node
          sink = int(data[3])  # downstream node
          sourceObject = self.nodeDict[source]
          sinkObject = self.nodeDict[sink]
          # create the link using params, and specify upstream downstream nodes
          self.linkDict[int(data[0])] = linkFactory(getattr(linkModel, data[1]),\
                          int(data[0]), sourceObject, sinkObject, params )
    except IOError as ioerr:
      print('... error reading link file: ' + str(ioerr) + ' ...')

    self.setNodeAdjacency()
    return None  
  
  def setNodeAdjacency(self):
    """
    puts link objects in node classes upstreamLinks and downstreamLinks
    """
    for nodeID in self.nodeDict:
      node = self.nodeDict[nodeID]
      for linkID in node.fstar:
        node.downstreamLinks.append(self.linkDict[linkID])
      for linkID in node.rstar:
        node.upstreamLinks.append(self.linkDict[linkID])
    return None
  
  def readDemand(self, dFile):
    """
    read OD file, tab delimited demand file
    make sure time starts from zero
    """
    if self.nodeDict == {}:
      raise Exception('... read nodes before links ...')
    
    listofOrigins = list()
    try:
      with open(dFile) as df:
        print('... reading demand file ...')
        df.readline()  # skips the first line
        for line in df:
          data = line.strip().split('\t')
          time = int(data[0])  # based format of text file
          if data[1] != '[]':
            origins = data[1].strip('[]').split(',')
            demand = data[2].strip('[]').split(',')
            for key, origin in enumerate(origins):
              self.nodeDict[int(origin)].demandRates[time] = float(demand[key])
              if int(origin) not in listofOrigins:
                listofOrigins.append(int(origin))
          
    except IOError as ioerr:
      print('... error reading OD file: ' + str(ioerr) + ' ...')
      
    self._completeZoneSetup(listofOrigins)
    return None
  
  def _completeZoneSetup(self, listofOrigins):
    """
    assigns demandRates of zero to zones when there's no flow
    """
    for nodeID in listofOrigins:
      node = self.nodeDict[nodeID]
      for time in self.totalTimesteps:
        if time not in node.demandRates:
          node.demandRates[time] = 0    
    return None
  
  def loadNetworkStep(self, time):
    """
    implements the network loading algorithm for one time step
    and returns the densities on the cells
    """
    for nodeID in self.nodeDict:
      self.nodeDict[nodeID].nodeUpdate(time, self.timeStep)
    for linkID in self.linkDict:
      self.linkDict[linkID].linkUpdate(time + 1)
    
    linkDensities = list()
    listofLinkDensities = list()
    for linkID in self.linkDict:
      if linkID is not 9:
        listofDensitiesPerLink = self.linkDict[linkID].linkDensity()
        listofLinkDensities.append(listofDensitiesPerLink)
        for linkdenval in listofDensitiesPerLink:
          linkDensities.append(linkdenval)
    return linkDensities, listofLinkDensities
  
  def networkLoading(self):
    """
    the full network loading algorithm, returns dictionary of
    dictionaries where outer keys are time, inner key is link, and value
    is the densities on the cells
    """
    self.netLoadingResults = dict()
    for time in self.totalTimesteps:
      self.netLoadingResults[time] = self.loadNetworkStep(time)
      
    return self.netLoadingResults
  
  def resetCounts(self):
    for linkID in self.linkDict:
      link = self.linkDict[linkID]
      link._upstreamCounts = dict()
      link._downstreamCounts = dict()
      link.inFlow = 0
      link.outFlow = 0
    return None

if __name__ == '__main__':
    simTime = 4490
    simTimeStep = 10
    totalTimeSteps = range(int(np.ceil(float(simTime)/simTimeStep)) + 1)
    linkfile = 'VISSIMnetwork/links.txt'
    nodefile = 'VISSIMnetwork/nodes.txt'
    demandfile = 'VISSIMnetwork/demand6600.txt'
    trafficNet = Network(simTime, simTimeStep, nodefile, linkfile, demandfile)
    results=trafficNet.networkLoading()
    print(results)