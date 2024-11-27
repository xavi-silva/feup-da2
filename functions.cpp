#include "functions.h"
#include "haversine.h"
#include <list>
#include <stack>
#include <iostream>
#include <chrono>


using namespace std;

using namespace std::chrono;
/*
4.1. Backtracking Algorithm [T2.1: 4 points]
In this approach, you are asked to develop a backtracking algorithmic approach to the TSP for a graph
starting and ending the node tour on node labelled with the zero-identifier label. You are expected to use
the small graphs to validate this approach and to observe that for the larger graphs this approach is not
really feasible. To this extent, you are suggested to plot the execution time (or other performance metrics
you find significant) to illustrate the feasibility or not of this approach for larger graphs.
*/

void TSPBackTracking(Graph<int>& graph, int currentNode, std::vector<bool>& visited, std::vector<int>& path, std::vector<int>& optimalPath, double& minCost, double& currentCost, int& count) {
    int nodes = graph.getNumVertex();
    path.push_back(currentNode);
    visited[currentNode] = true;
    count++;

    if (path.size() == nodes) {
        for (auto edge : graph.findVertex(currentNode)->getAdj()) {
            if (edge->getDest()->getInfo() == path[0]) { // Check if we can return to the start
                double returnCost = currentCost + edge->getWeight();
                if (returnCost < minCost) {
                    minCost = returnCost;
                    optimalPath = path; // Update the optimal path
                }
                break;
            }
        }
    } else {
        for (auto edge : graph.findVertex(currentNode)->getAdj()) {
            int nextNode = edge->getDest()->getInfo();
            if (!visited[nextNode]) {
                visited[nextNode] = true;
                double newCost = currentCost + edge->getWeight();

                if (newCost < minCost) { // Pruning step
                    TSPBackTracking(graph, nextNode, visited, path, optimalPath, minCost, newCost, count);
                }
                visited[nextNode] = false;
            }
        }
    }

    path.pop_back(); // Remove the current node from the path as we backtrack
    visited[currentNode] = false;
}

std::vector<Vertex<int>*> solveTSPBacktracking(Graph<int>& graph, int startNode) {
    int nodes = graph.getNumVertex();
    std::vector<bool> visited(nodes, false);
    std::vector<int> path;
    std::vector<int> optimalPath;
    double minCost = std::numeric_limits<double>::max();
    double cost = 0;
    int count = 0;

    visited[startNode] = true;
    TSPBackTracking(graph, startNode, visited, path, optimalPath, minCost, cost, count);
    /*
    std::cout << "Minimum cost: " << minCost << std::endl;
    std::cout << "Optimal path: ";
    for (int node : optimalPath) {
        std::cout << node << " ";
    }
    std::cout << std::endl;
    std::cout << "Number of recursive calls: " << count << std::endl;
    */
    std::vector<Vertex<int>*> resultPath;
    for (int nodeId : optimalPath) {
        resultPath.push_back(graph.findVertex(nodeId));
    }

    resultPath.push_back(graph.findVertex(startNode));
    return resultPath;
}


/*4.4*/

bool isConnected(Graph<int>& graph, int startNode, std::unordered_set<int>& reachableNodes){
    queue<int> q;
    q.push(startNode);
    reachableNodes.insert(startNode);

    while(!q.empty()){
        int current = q.front();
        q.pop();

        for(auto edge : graph.findVertex(current)->getAdj()){
            int neighbor = edge->getDest()->getInfo();
            if(reachableNodes.find(neighbor) == reachableNodes.end()){
                reachableNodes.insert(neighbor);
                q.push(neighbor);
            }
        }
    }

    return (reachableNodes.size() == graph.getNumVertex());
}

std::vector<std::vector<double>> calculateShortestPaths(Graph<int>& graph, const std::unordered_set<int>& nodes){
    int numV = graph.getNumVertex();
    vector<vector<double>> dist(numV, vector<double>(numV, std::numeric_limits<double>::max()));

    for(auto node : nodes){
        priority_queue<pair<double, Vertex<int>*>, vector<pair<double, Vertex<int>*>>,greater<>> priorityQueue;
        priorityQueue.push({0, graph.findVertex(node)});
        dist[node][node] = 0;

        while(!priorityQueue.empty()){
            auto [d,v] = priorityQueue.top();
            priorityQueue.pop();

            if(d > dist[node][v->getInfo()]) continue;

            for (auto edge : v->getAdj()){
                auto next = edge->getDest();
                double newDist = d + edge->getWeight();
                if(newDist < dist[node][next->getInfo()]){
                    dist[node][next->getInfo()] = newDist;
                    priorityQueue.push({newDist, next});
                }
            }
        }
    }
    return dist;
}

std::vector<int> tspNearestNeighbor(Graph<int>& graph, int startNode, const std::unordered_set<int>& nodes, double& totalCost){
    vector<int> path;
    unordered_set<int> visited;
    int currentNode = startNode;

    path.push_back(currentNode);
    visited.insert(currentNode);
    totalCost = 0;

    while(visited.size() < nodes.size()){
        double minDist = numeric_limits<double>::max();
        int nextNode = -1;

        for(auto edge : graph.findVertex(currentNode)->getAdj()){
            int neighbor = edge->getDest()->getInfo();
            if(visited.find(neighbor) == visited.end() && edge->getWeight() < minDist){
                minDist = edge->getWeight();
                nextNode = neighbor;
            }
        }

        if(nextNode == -1){
            std::cout << "Error: Could not find a path to cover all nodes." << std::endl;
            return {};
        }

        path.push_back(nextNode);
        visited.insert(nextNode);
        totalCost += minDist;
        currentNode = nextNode;
    }

    // Ensure return to the starting node to complete the tour
    for (auto edge : graph.findVertex(currentNode)->getAdj()) {
        if (edge->getDest()->getInfo() == startNode) {
            path.push_back(startNode);
            totalCost += edge->getWeight();
            break;
        }
    }

    // If the path does not return to the start, add the return manually
    if (path.back() != startNode) {
        totalCost += calculateShortestPaths(graph, nodes)[currentNode][startNode];
        path.push_back(startNode);
    }

    return path;
}

std::vector<Vertex<int>*> solveRealWorldTSP(Graph<int>& graph, int startNode){
    std::unordered_set<int> reachableNodes;
    isConnected(graph, startNode, reachableNodes);
    auto shortestPaths = calculateShortestPaths(graph, reachableNodes);
    double totalCost = 0;
    auto tspPath = tspNearestNeighbor(graph, startNode, reachableNodes, totalCost);

    if(tspPath.empty() || tspPath.size() != reachableNodes.size() + 1) {
        std::cout << "The nearest neighbor heuristic did not find a complete tour." << std::endl;
        return {};
    }
    /*
    std::cout << "TSP Path: ";
    for(size_t i = 0; i < tspPath.size(); i++){
        std::cout << tspPath[i];
        if(i < tspPath.size() - 1 ){
            std::cout << "->";
        }
    }
    std::cout << std::endl;
    std::cout << "Total Cost: " << totalCost << std::endl;
    */
    std::vector<Vertex<int>*> resultPath;
    for (int nodeId : tspPath) {
        resultPath.push_back(graph.findVertex(nodeId));
    }

    return resultPath;
}

// Function implementing Prim's algorithm to find the minimum spanning tree (MST) of a graph
std::vector<Vertex<int> *> prim(Graph<int> * g, int currentNode) {
// Check if the graph is empty
    if (g->getVertexSet().empty()) {
        return g->getVertexSet(); // Return an empty set if the graph is empty
    }
// Initialize the vertices in the graph
    for(auto v : g->getVertexSet()) {
        v->setDist(INF); // Set distance to infinity
        v->setPath(nullptr); // Set path to null
        v->setVisited(false); // Mark as not visited
    }
// Select the first vertex as the starting point
    Vertex<int>* s = g->findVertex(currentNode);
    s->setDist(0); // Set distance of the starting vertex to 0
// Priority queue to store vertices based on their distances
    MutablePriorityQueue<Vertex<int>> q;
    q.insert(s);
// Main loop for the Prim's algorithm
    while( ! q.empty() ) {
// Extract the vertex with the minimum distance from the priority queue
        auto v = q.extractMin();
        v->setVisited(true); // Mark the vertex as visited
// Iterate through the adjacent edges of the current vertex
        for(auto &e : v->getAdj()) {
            Vertex<int>* w = e->getDest(); // Get the destination vertex of the edge
// Check if the destination vertex is not visited
            if (!w->isVisited()) {
                auto oldDist = w->getDist(); // Get the current distance of the destination vertex
// Check if the weight of the edge is less than the current distance of the destination vertex
                if(e->getWeight() < oldDist) {
                    w->setDist(e->getWeight()); // Update the distance of the destination vertex
                    w->setPath(e); // Update the path to the current edge
// If the destination vertex had infinite distance, insert it into the priority queue
                    if (oldDist == INF) {
                        q.insert(w);
                    }
// If the destination vertex had finite distance, decrease its key in the priority queue
                    else {
                        q.decreaseKey(w);
                    }
                }
            }
        }
    }
// Return the set of vertices after the Prim's algorithm completes
    return g->getVertexSet();
}


void normalizeGraph(Graph<int>* graph){
    bool isAdj = false;
    for (auto v1 : graph->getVertexSet()){
        for (auto v2 : graph->getVertexSet()){
            if (v1->getInfo() != v2->getInfo()){
                for (auto e : v1->getAdj()){
                    if (e->getDest() == v2) isAdj = true;
                }
                if (!isAdj){
                    double w = calculate_distance(v1->getLat(),v1->getLong(),v2->getLat(),v2->getLong());
                    graph->addEdge(v1->getInfo(),v2->getInfo(),w);
                    cout << "added edge (" << v1->getInfo() << "," << v2->getInfo() << ")" << endl;
                }
                isAdj = false;
            }
        }
    }
}

/*
    Approx-TSP-Tour(G, c)
    select any r in V as “root” vertex
    compute MST T rooted at r
    L = pre-order walk of T
    define tour H that visits G using order in L skipping to
    the next unvisted node if needed.
    return H
 */
vector<Vertex<int>*> solveTSPTriangularInequality(Graph<int>& g, int currentNode){
    for(auto v : g.getVertexSet()) v->setVisited(false);
    auto mst = prim(&g, currentNode);

    for(auto v : g.getVertexSet()) v->setVisited(false);

    vector<Vertex<int>*> path;
    stack<Vertex<int>*> stack;
    Vertex<int>* prev = g.findVertex(currentNode);
    stack.push(prev);
    vector<Edge<int>*> adj;

    while (!stack.empty()){
        prev = stack.top();
        prev->setVisited(true);
        stack.pop();
        path.push_back(prev);

        adj = prev->getAdj();
        sort(adj.begin(), adj.end(), [](Edge<int>* a, Edge<int>* b){
            return a->getWeight() > b->getWeight();
        });
        for (auto e : adj){
            auto next = e->getDest();
            if (!next->isVisited() && next->getPath()==e){
                stack.push(next);
            }

        }
    }
    path.push_back(g.findVertex(currentNode));
    return path;
}




/* Inconsistent results and 5min+ execution time on Real World graph1

void solveTSPTriangularInequality(Graph<int>& g, int currentNode){
    // Timer for execution time
    auto start = high_resolution_clock::now();

    vector<Vertex<int>*> path;
    prim(&g, currentNode);
    auto prev = g.findVertex(currentNode);
    path.push_back(prev);

    for (auto v:g.getVertexSet()) {
        v->setVisited(false);
        v->setDist(INT_MAX);
    }

    vector<Vertex<int>*> neighbours;
    stack<Vertex<int>*> stack;
    bool hasPath = false;

    for (int i = 0; i < g.getVertexSet().size() - 1; i++){

        cout << prev->getInfo() << endl;
        for (auto e : prev->getAdj()){

            auto v = e->getDest();
            if (v->getInfo() != 0){
                cout << v->getPath()->getDest()->getInfo()  << " " << (v->getPath()->getDest() == prev || v->getPath()->getOrig() == prev) << endl;
                if ((v->getPath()->getDest() == prev || v->getPath()->getOrig() == prev) && !prev->isVisited()) {
                    cout << "entra" << endl;
                    v->setDist(v->getPath()->getWeight());
                    neighbours.push_back(v);
                    hasPath = true;
                }
            }
        }

        if (hasPath){
            sort(neighbours.begin(), neighbours.end(), [](Vertex<int>* a, Vertex<int>* b) {
                return b < a;
            });
            path.push_back(neighbours.back());
            prev->setVisited(true);
            prev = neighbours.back();
            neighbours.pop_back();
            while(!neighbours.empty()){
                stack.push(neighbours.back());
                neighbours.pop_back();
            }
        }
        else{
            if (stack.empty()) cout << "Stack empty" << endl;
            prev->setVisited(true);
            path.push_back(stack.top());
            prev = stack.top();
            stack.pop();
        }
        hasPath = false;
    }
    path.push_back(g.findVertex(currentNode));

    for (auto x : path){
        cout << x->getInfo() << " ";
    }

    cout << "Triangular Inequality Cost: " << calculatePathCost(g, path) << endl;
    // Execution time
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    cout << "Execution Time: " << duration.count() << " milliseconds" << endl;
}
*/


//4.3
Graph<int> formMST(Graph<int> g, int currentNode) {
    Graph<int> mst;

    // Adiciona todos os vértices ao MST
    for (auto v : g.getVertexSet()) {
        mst.addVertex(v->getInfo());
    }

    // Adiciona arestas ao MST
    for (auto v : g.getVertexSet()) {
        if (v->getInfo() != currentNode) {
            auto edge = v->getPath();

            // Verifica se edge não é nulo antes de acessar seus membros
            if (edge != nullptr) {
                mst.addBidirectionalEdge(edge->getOrig()->getInfo(), edge->getDest()->getInfo(), edge->getWeight());
            }
        }
    }

    return mst;
}


vector<int> findOddDegreeVertices(const Graph<int>& graph) {
    vector<int> oddVertices;
    for(auto v : graph.getVertexSet()){
        if(v->getAdj().size() % 2 != 0){
            oddVertices.push_back(v->getInfo());
        }
    }
    return oddVertices;
}

bool isOdd(const Vertex<int>& v) {
    return v.getAdj().size() % 2 != 0;
}

double getEdgeWeight(const Graph<int>& graph, int u, int v) {

    auto x = graph.findVertex(u);
    for(auto edge : x->getAdj()){
        if(edge->getDest()->getInfo() == v){
            return edge->getWeight();
        }
    }
    return 0; // or an appropriate value indicating no such edge exists
}

Graph<int> MinWeightPerfMatch(Graph<int>& graph, const std::vector<int>& odders) {
    Graph<int> grafo;
    for(int odd : odders) { // For each odd vertex
        stack<Vertex<int>> pilha;
        auto v = graph.findVertex(odd);
        if(grafo.findVertex(v->getInfo()) == nullptr) {
            grafo.addVertex(v->getInfo());
        }
        if(!v->isVisited()) { // If not visited
            v->setVisited(true); // Mark as visited
            int min = INT_MAX;
            for(auto e : v->getAdj()) { // Look through adjacent edges
                if(isOdd(*e->getDest()) && !e->getDest()->isVisited()) { // Only if the destination is also odd and not visited
                    if(e->getWeight() < min) { // If the weight is minimum
                        if(!pilha.empty()) pilha.pop(); // Remove edge with minimum weight
                        pilha.emplace(e->getDest()->getInfo()); // Update
                    }
                }
            }
            if(!pilha.empty()){
                grafo.addVertex(pilha.top().getInfo()); // Add vertex to the graph
                grafo.addEdge(v->getInfo(), pilha.top().getInfo(), getEdgeWeight(graph, v->getInfo(), pilha.top().getInfo())); // Create the edge
                graph.findVertex(pilha.top().getInfo())->setVisited(true); // Mark the last vertex as visited
            }}
    }

    return grafo;
}

vector<Vertex<int>*> findEulerianPath(Graph<int>& graph, int start) {
    vector<Vertex<int>*> path;
    stack<Vertex<int>*> currPath;
    currPath.push(graph.findVertex(start));

    while (!currPath.empty()) {
        auto u = currPath.top();
        if (u->getAdj().empty()) {
            path.push_back(u);
            currPath.pop();
        } else {
            auto v = u->getAdj().front()->getDest();
            currPath.push(v);
            graph.removeEdge(u->getInfo(), v->getInfo());
            graph.removeEdge(v->getInfo(), u->getInfo()); // Se for um grafo não direcionado
        }
    }

    reverse(path.begin(), path.end());
    return path;
}

vector<Vertex<int>*> makeHamiltonianCycle(const vector<Vertex<int>*> eulerianPath) {
    vector<Vertex<int>*> hamiltonianCycle;
    unordered_set<int> visited;

    for (auto vertex : eulerianPath) {
        if (visited.find(vertex->getInfo()) == visited.end()) {
            hamiltonianCycle.push_back(vertex);
            visited.insert(vertex->getInfo());
        }
    }

    // Fechando o ciclo ao retornar ao vértice inicial
    hamiltonianCycle.push_back(hamiltonianCycle.front());

    return hamiltonianCycle;
}

Graph<int> combineGraphs(const Graph<int>& mst, const Graph<int>& matching) {
    Graph<int> eulerianMultigraph = mst; // Começa com a MST

    // Adiciona todas as arestas do emparelhamento perfeito mínimo
    for (auto v : matching.getVertexSet()) {
        for (auto e : v->getAdj()) {
            eulerianMultigraph.addEdge(v->getInfo(), e->getDest()->getInfo(), e->getWeight());
        }
    }

    return eulerianMultigraph;
}

vector<Vertex<int>*> christofidesTsp(Graph<int>& graph, int currentNode) {
    vector<Vertex<int>*> optimalPath;
    // Passo 1: Encontrar a MST
    prim(&graph, currentNode);
    auto mst = formMST(graph,currentNode);
    // Passo 2: Identificar vértices de grau ímpar
    auto oddVertices = findOddDegreeVertices(mst);

    // Passo 3: Construir emparelhamento perfeito mínimo
    auto matching = MinWeightPerfMatch(mst,oddVertices);

    // Passo 4: Combinar MST com emparelhamento perfeito
    Graph<int> eulerianMultigraph = combineGraphs(mst, matching);

    // Passo 5: Encontrar ciclo euleriano
    auto eulerianPath = findEulerianPath(eulerianMultigraph, eulerianMultigraph.getVertexSet().front()->getInfo());

    // Passo 6: Transformar ciclo euleriano em passeio hamiltoniano
    optimalPath = makeHamiltonianCycle(eulerianPath);

    return optimalPath;
}
