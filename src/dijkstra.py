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
        @param ways: node connections to others list. [-1] if all the ways are are closed.
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
class Graph:

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
        if parent[j] == -1:
            print('{}-'.format(j)),
            return
        self.printPath(parent, parent[j])
        print('{}-'.format(j))

    # A utility function to print
    # the constructed distance
    # array
    def printSolution(self, dist, parent):
        src = 0
        print("Vertex \t\tDistance from Source \t\tPath")
        for i in range(1, len(dist)):
            print("\n%d --> %d \t\t%d \t\t\t\t\t" % (src, i, dist[i])),
            self.printPath(parent, i)



    def dijkstra(self, graph, src):

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
        dist[src] = 0

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
            for i in range(col):

                if graph[u][i] and i in queue:
                    if dist[u] + graph[u][i] < dist[i]:
                        dist[i] = dist[u] + graph[u][i]
                        parent[i] = u

        # print the constructed distance array
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
        if j not in all_nodes[i].ways() or all_nodes[i].ways == -1:
            matrix[i][j] = 0
        else: 
            matrix[i][j] = np.round(Distance(all_nodes[i].get_position(),all_nodes[j].get_position()),2)

print(matrix)
# Print the solution
g = Graph()
g.dijkstra(matrix, 0)