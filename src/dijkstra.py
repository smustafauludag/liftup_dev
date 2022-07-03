#!/usr/bin/env python

"""
Dijkstra algorithm
@authors: Akif Canatan, Osman ozkal, Sefer Mustafa Uludag
@mails: akif.canitin@gmail.com , osmanozkal06@gmail.com, smustafauludag@gmail.com
"""

from vehicle import Distance
import numpy as np



class Node(object):
  """
  Node class for dijkstra
  """
  def __init__(self,position,id,ways):
    """
    @param position: [x,y]
    @param id: node_id
    @param ways: list of connections to other nodes
    """
    self._pos = position
    self._id = id
    self._way = np.array(ways)
    
  def get_position(self):
    return self._pos
  
  def get_id(self):
    return self._id
  
  def ways(self):
    return self._way


# Class to represent a graph
class Graph():

  def __init__(self,src,target):
    self.nodes = []
    self.src = src
    self.target = target
    # A utility function to find the
    # vertex with minimum dist value, from
    # the set of vertices still in queue
  def minDistance(self, dist, queue):
    # Initialize min value and min_index as -1
    minimum = float("Inf")
    min_index = -1

    # from the dist array,pick one which
    # has min value and is till in queue
    for i in range(len(dist)):
      if dist[i] < minimum and i in queue:
        minimum = dist[i]
        min_index = i
    return min_index

    # Function to print shortest path
    # from source to j
    # using parent array
  def printPath(self, parent, j):

    # Base Case : If j is source
    if parent[j] == -1: return
    self.printPath(parent, parent[j])
    self.nodes.append(j)


    # A utility function to print
    # the constructed distance
    # array
  def printSolution(self, dist, parent):
    path = "{}".format(self.src)
    print("\nVertex \t\tDistance from Source \tPath")
    print("""\n{} --> {} \t\t{} \t\t""".format(self.src, self.target, dist[self.target])),
    self.printPath(parent, self.target)
    for i in self.nodes:
      path = path+ "-{}".format(i)
    print(path)


  def dijkstra(self, graph):

    row = len(graph)
    col = len(graph[0])

    # The output array. dist[i] will hold
    # the shortest distance from src to i
    # Initialize all distances as INFINITE
    dist = [float("Inf")] * row

    # Parent array to store
    # shortest path tree
    parent = [-1] * row

    # Distance of source vertex
    # from itself is always 0
    dist[self.src] = 0

    # Add all vertices in queue
    queue = []
    for i in range(row):
      queue.append(i)

    # Find shortest path for all vertices
    while queue:

      # Pick the minimum dist vertex
      # from the set of vertices
      # still in queue
      u = self.minDistance(dist, queue)

      # remove min element
      queue.remove(u)

      # Update dist value and parent
      # index of the adjacent vertices of
      # the picked vertex. Consider only
      # those vertices which are still in
      # queue
      """ Relaxation """
      for i in range(col):
        if graph[u][i] and i in queue:
          if dist[u] + graph[u][i] < dist[i]:
            dist[i] = round(dist[u] + graph[u][i],2)
            parent[i] = u
    self.printSolution(dist, parent)

node_0 = Node(position=[0,0],id=0,ways=[1,5])
node_1 = Node(position=[2,1],id=1,ways=[0,2,3,4,5])
node_2 = Node(position=[5,1],id=2,ways=[1,3,6,8])
node_3 = Node(position=[4,2],id=3,ways=[1,2,4,6])
node_4 = Node(position=[3,3],id=4,ways=[1,3,5,6,7])
node_5 = Node(position=[1,4],id=5,ways=[0,1,4,7])
node_6 = Node(position=[4,4],id=6,ways=[2,3,4,7,8])
node_7 = Node(position=[3,5],id=7,ways=[4,5,6,8])
node_8 = Node(position=[5,5],id=8,ways=[2,6,7])

all_nodes = [node_0,
             node_1,
             node_2,
             node_3,
             node_4,
             node_5,
             node_6,
             node_7,
             node_8]
# Create graph
matrix = np.zeros((len(all_nodes),len(all_nodes)))
for i in range (0,len(all_nodes)):
  for j in range (0,len(all_nodes)):
    if j not in all_nodes[i].ways():
      matrix[i][j] = np.inf
    else: 
      matrix[i][j] = np.round(Distance(all_nodes[i].get_position(),all_nodes[j].get_position()),2)
# Print the solution
g = Graph(src=0,target=8)
g.dijkstra(matrix)
