#include <iostream>
#include "dataStructures/Graph.h"
#include "dataParser.h"
#include "functions.h"
#include "menu.h"
#include <set>


int main() {
    //Graph<int> graph;
    //ToyGraphParser("shipping.csv", graph);//error not fully connected
    //ToyGraphParser("tourism.csv", graph);
    //ToyGraphParser("stadiums.csv", graph);
   //RealWorldGraphParser("graph2", graph);
   //solveTSPBacktracking(graph, 0);

    // Depois para apresentar damos logo load aos 3 grafos
    //Graph<int> toyGraph;
    //Graph<int> extraGraph;
    //Graph<int> realGraph;

    Menu menu;
    menu.start();

    //ExtraFullyConnectedGraphsParser("edges_500.csv", graph, 500);
    //RealWorldGraphParser("graph1", graph); // 1.14311e+06 - 5 min :(
    //normalizeGraph(&graph);

    //solveTSPBacktracking(graph, 0);
    //solveTSPTriangularInequality(graph, 0);
    return 0;
}
