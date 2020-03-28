# Simple Directed Graph Class for Waypoint Graphs
# Tung M. Phan
# California Institute of Technology
# May 9th, 2018
import numpy as np
import random

class DirectedGraph():
    def __init__(self):
        self._nodes = [] # list of nodes
        self._edges = dict() # set of edges is a dictionary of sets (of nodes)
        self._sources = []
        self._sinks = []
        self._predecessors = dict() # predecessors of nodes

    def add_node(self, node): # add a node
            self._nodes.append(node)

    def add_source(self, source): # add a source node
            self._sources.append(source)

    def add_sink(self, sink): # add a source node
            self._sinks.append(sink)

    def add_edges(self, edge_set): # add edges
        for edge in edge_set:
            if len(edge) != 2:
                raise SyntaxError('Each edge must be a 2-tuple of the form (start, end)!')
            for node in edge:
                if node not in self._nodes:
                    self.add_node(node)
            try: self._edges[edge[0]].add(edge[1])
            except KeyError:
                self._edges[edge[0]] = {edge[1]}
            try: self._predecessors[edge[1]].add(edge[0])
            except KeyError:
                self._predecessors[edge[1]] = {edge[0]}

    def add_double_edges(self, edge_set): # add two edges (of the same weight) for two nodes
        for edge in edge_set:
            self.add_edges([edge])
            self.add_edges([[edge[1], edge[0]] + edge[2:]])

    def print_graph(self):
        print('The directed graph has ' + str(len(self._nodes)) + ' nodes: ')
        print(str(list(self._nodes)).strip('[]'))
        print('and ' + str(sum([len(self._edges[key]) for key in self._edges])) + ' edges: ')
        for start_node in self._edges:
            print(str(start_node) + ' -> ' +  str(list(self._edges[start_node])).strip('[]'))

class WeightedDirectedGraph(DirectedGraph):
    def __init__(self):
        DirectedGraph.__init__(self)
        self._weights = dict() # a dictionary of weights
        self._edge_labels = dict()

    def add_edges(self, edge_set, label_edges = False, edge_label_set = None, use_euclidean_weight=False): # override parent's method to allow for edge weights
        '''
        Use this function to add edges to the directed graph. When 'use_euclidean_weight' is False, each edge must be a 3-tuple of the form (start, end, weight), otherwise the weight will be automatically computed as the euclidean distances between the nodes (which are assumed to be points in a 2D plane)

        '''
        for idx, edge in enumerate(edge_set):
            if use_euclidean_weight:
                if len(edge) != 2:
                    raise SyntaxError('Each edge must be a 2-tuple of the form (start, end) where start and end contain coordinates of points in a 2D plane!')
                for node in edge[0:2]:
                    if node not in self._nodes:
                        self.add_node(node)
                try: self._edges[edge[0]].add(edge[1])
                except KeyError:
                    self._edges[edge[0]] = {edge[1]}
                try: self._predecessors[edge[1]].add(edge[0])
                except KeyError:
                    self._predecessors[edge[1]] = {edge[0]}
                x = np.array([edge[0][-2], edge[0][-1]], float) # need to cast to float, otherwise numerical precision errors may occur
                y = np.array([edge[1][-2], edge[1][-1]], float) # same as above
                self._weights[(edge[0], edge[1])] = np.linalg.norm(x-y)  # add Euclidean distance as weight
            else:
                if len(edge) != 3:
                    raise SyntaxError('Each edge must be a 3-tuple of the form (start, end, weight)!')
                for node in edge[0:2]:
                    if node not in self._nodes:
                        self.add_node(node)
                try: self._edges[edge[0]].add(edge[1])
                except KeyError:
                    self._edges[edge[0]] = {edge[1]}
                try: self._predecessors[edge[1]].add(edge[0])
                except KeyError:
                    self._predecessors[edge[1]] = {edge[0]}
                self._weights[(edge[0], edge[1])] = edge[2] # add weight
            if label_edges:
                if len(edge) == 3:
                    edge = (edge[0], edge[1])
                self._edge_labels[edge] = edge_label_set[idx]

    def print_graph(self):
        print('The directed graph has ' + str(len(self._nodes)) + ' nodes: ')
        print(str(list(self._nodes)).strip('[]'))
        print('and ' + str(sum([len(self._edges[key]) for key in self._edges])) + ' edges: ')
        for start_node in self._edges:
            for end_node in self._edges[start_node]:
                print(str(start_node) + ' -(' + str(self._weights[start_node, end_node]) +  ')-> ' +  str(end_node))
