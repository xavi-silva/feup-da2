#ifndef UNTITLED_FUNCTIONS_H
#define UNTITLED_FUNCTIONS_H

#include "dataStructures/Graph.h"
#include "haversine.h"
#include <vector>
#include <set>
#include <iostream>
#include <unordered_set>
using namespace std;

void normalizeGraph(Graph<int>* graph);
std::vector<Vertex<int> *> prim(Graph<int> * g, int currentNode);

/*4.1*/
void TSPBackTracking(Graph<int>& graph, int currentNode, vector<bool>& visited, vector<int>& path, double& minCost, double& currentCost, int& count);
std::vector<Vertex<int>*> solveTSPBacktracking(Graph<int>& graph, int startNode);

/*4.2*/
vector<Vertex<int>*> solveTSPTriangularInequality(Graph<int>& graph, int currentNode);

/*4.3*/
Graph<int> formMST(Graph<int> g, int currentNode);
vector<int> findOddDegreeVertices(const Graph<int>& graph);
bool isOdd(const Vertex<int>& v);
double getEdgeWeight(const Graph<int>& graph, int u, int v);
Graph<int> MinWeightPerfMatch(Graph<int>& graph, const std::vector<int>& odders);
vector<Vertex<int>*> findEulerianPath(Graph<int>& graph, int start);
vector<Vertex<int>*> makeHamiltonianCycle(const vector<Vertex<int>*> eulerianPath);
Graph<int> combineGraphs(const Graph<int>& mst, const Graph<int>& matching);
vector<Vertex<int>*> christofidesTsp(Graph<int>& graph, int currentNode);



/* 4.4 */
bool isConnected(Graph<int>& graph, int startNode, std::unordered_set<int>& reachableNodes);
std::vector<std::vector<double>> calculateShortestPaths(Graph<int>& graph, const std::unordered_set<int>& nodes);
std::vector<int> tspNearestNeighbor(Graph<int>& graph, int startNode, const std::unordered_set<int>& nodes);
std::vector<Vertex<int>*> solveRealWorldTSP(Graph<int>& graph, int startNode);


#endif //UNTITLED_FUNCTIONS_H

