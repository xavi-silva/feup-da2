#include <vector>
#include <iostream>
#include <limits>
#include "dataParser.h"
#include "functions.h"
#include <fstream>
#include <chrono>
#ifndef UNTITLED_MENU_H
#define UNTITLED_MENU_H

#endif //UNTITLED_MENU_H

std::ofstream file("../output.txt");

class Menu {
public:
    void start();
private:
    void chooseDataSet(int datasetGroup);
    void chooseAlgorithm(Graph<int> *g, const int datasetGroup, const int dataset);
    void invalidInputHandler(std::vector<int> inputs, int last);
    void outputPath(Graph<int> g, vector<Vertex<int>*> path, int alg, int datasetgroup, int dataset, int duration);
    double calculatePathCost(Graph<int> &g, vector<Vertex<int>*> path);

    };

void Menu::start() {

    while(true){
        std::cout << "Choose a group of datasets" << std::endl
                  << "1. Toy Graphs" << std::endl
                  << "2. Extra Fully Connected Graphs" << std::endl
                  << "3. Real World Graphs" << std::endl
                  << "0. Quit service" << std::endl;

        int datasetGroup;
        while(!(std::cin >> datasetGroup) || (datasetGroup < 0 || datasetGroup > 3)){
            invalidInputHandler({0,1,2},3);
        }
        if (datasetGroup == 0) return;
        chooseDataSet(datasetGroup);
    }
}

void Menu::chooseDataSet(const int datasetGroup){
    Graph<int> g;
    int dataset;
    switch (datasetGroup) {
        case 1:
            std::cout << "1. Shipping" << std::endl
                      << "2. Stadiums" << std::endl
                      << "3. Tourism" << std::endl
                      << "0. Go back" << std::endl;
            while(!(std::cin >> dataset) || (dataset < 0 || dataset > 3)){
                invalidInputHandler({0,1,2},3);
            }
            switch(dataset){
                case 0: return;
                case 1:
                    ToyGraphParser("shipping.csv",g);
                    break;
                case 2:
                    ToyGraphParser("stadiums.csv",g);
                    break;
                case 3:
                    ToyGraphParser("tourism.csv",g);
                    break;
                default:
                    break;
            }
            break;
        case 2:
            std::cout << "1. 25 edges" << std::endl
                      << "2. 50 edges" << std::endl
                      << "3. 75 edges" << std::endl
                      << "4. 100 edges" << std::endl
                      << "5. 200 edges" << std::endl
                      << "6. 300 edges" << std::endl
                      << "7. 400 edges" << std::endl
                      << "8. 500 edges" << std::endl
                      << "9. 600 edges" << std::endl
                      << "10. 700 edges" << std::endl
                      << "11. 800 edges" << std::endl
                      << "12. 900 edges" << std::endl;
            while(!(std::cin >> dataset) || (dataset < 0 || dataset > 12)){
                invalidInputHandler({0,1,2,3,4,5,6,7,8,9,10,11},12);
            }
            switch(dataset){
                case 0: return;
                case 1:
                    ExtraFullyConnectedGraphsParser("edges_25.csv",g,25);
                    break;
                case 2:
                    ExtraFullyConnectedGraphsParser("edges_50.csv",g,50);
                    break;
                case 3:
                    ExtraFullyConnectedGraphsParser("edges_75.csv",g,75);
                    break;
                case 4:
                    ExtraFullyConnectedGraphsParser("edges_100.csv",g,100);
                    break;
                case 5:
                    ExtraFullyConnectedGraphsParser("edges_200.csv",g,200);
                    break;
                case 6:
                    ExtraFullyConnectedGraphsParser("edges_300.csv",g,300);
                    break;
                case 7:
                    ExtraFullyConnectedGraphsParser("edges_400.csv",g,400);
                    break;
                case 8:
                    ExtraFullyConnectedGraphsParser("edges_500.csv",g,500);
                    break;
                case 9:
                    ExtraFullyConnectedGraphsParser("edges_600.csv",g,600);
                    break;
                case 10:
                    ExtraFullyConnectedGraphsParser("edges_700.csv",g,700);
                    break;
                case 11:
                    ExtraFullyConnectedGraphsParser("edges_800.csv",g,800);
                    break;
                case 12:
                    ExtraFullyConnectedGraphsParser("edges_900.csv",g,900);
                    break;
                default:
                    break;
            }
            break;
        case 3:
            std::cout << "1. Graph 1" << std::endl
                      << "2. Graph 2" << std::endl
                      << "3. Graph 3" << std::endl
                      << "0. Go back" << std::endl;
            while(!(std::cin >> dataset) || (dataset < 0 || dataset > 3)){
                invalidInputHandler({0,1,2},3);
            }
            switch(dataset){
                case 0: return;
                case 1:
                    ToyGraphParser("graph1",g);
                    break;
                case 2:
                    ToyGraphParser("graph2",g);
                    break;
                case 3:
                    ToyGraphParser("graph3",g);
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
    chooseAlgorithm(&g, datasetGroup, dataset);
}

void Menu::chooseAlgorithm(Graph<int> *g, const int datasetGroup, const int dataset){
    vector<Vertex<int>*> path;
    while(true){
        std::cout << "Choose an algorithm:" << std::endl
                  << "1. Backtracking Algorithm" << std::endl
                  << "2. Triangular Approximation Heuristic" << std::endl
                  << "3. Other Heuristics" << std::endl
                  << "4. TSP in the Real World" << std::endl
                  << "0. Go back" << std::endl;

        int algorithm;
        while(!(std::cin >> algorithm) || (algorithm < 0 || algorithm > 4)){
            invalidInputHandler({0,1,2,3},4);
        }

        if (algorithm != 0){
            int node;
            std::cout << "Choose a starting node between 0 and " << g->getVertexSet().size()-1 << ": ";
            while(!(std::cin >> node) || (node < 0 || node > g->getVertexSet().size()-1)){
                invalidInputHandler({},0);
            }
        }


        auto start = std::chrono::high_resolution_clock::now();

        switch(algorithm){
            case 0: return;
            case 1:
                path = solveTSPBacktracking(*g, 0);
                break;
            case 2:
                normalizeGraph(g);
                path = solveTSPTriangularInequality(*g, 0);
                break;
            case 3:
                normalizeGraph(g);
                path = christofidesTsp(*g, 0);
                break;
            case 4:
                path = solveRealWorldTSP(*g, 0);
                break;
            default:
                break;
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        outputPath(*g, path, algorithm, datasetGroup, dataset, duration.count());

        std::string dummy;
        std::cout << "(Enter anything to continue):";
        std::cin >> dummy;
    }
}

void Menu::invalidInputHandler(std::vector<int> inputs, int last){
    if (last != 0) {
        std::cout << "Invalid input. Accepted inputs: ";
        for (int x: inputs) {
            std::cout << x << ", ";
        }
        std::cout << last << "." << std::endl;
    }
    else{
        std::cout << "Invalid input. Try again: ";
    }
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

void Menu::outputPath(Graph<int> g, vector<Vertex<int>*> path, int alg, int datasetgroup, int dataset, int duration){
    string data;
    switch(datasetgroup){
        case 1:
            switch(dataset){
                case 1:
                    data = "shipping.csv";
                            break;
                case 2:
                    data = "stadiums.csv";
                    break;
                case 3:
                    data = "tourism.csv";
                    break;
            }
            break;
        case 2:
            switch(dataset){
                case 1:
                    data = "edges_25.csv";
                    break;
                case 2:
                    data = "edges_50.csv";
                    break;
                case 3:
                    data = "edges_75.csv";
                    break;
                case 4:
                    data = "edges_100.csv";
                    break;
                case 5:
                    data = "edges_200.csv";
                    break;
                case 6:
                    data = "edges_300.csv";
                    break;
                case 7:
                    data = "edges_400.csv";
                    break;
                case 8:
                    data = "edges_500.csv";
                    break;
                case 9:
                    data = "edges_600.csv";
                    break;
                case 10:
                    data = "edges_700.csv";
                    break;
                case 11:
                    data = "edges_800.csv";
                    break;
                case 12:
                    data = "edges_900.csv";
                    break;
            }
            break;
        case 3:
            switch(dataset){
                case 1:
                    data = "Real World graph1";
                    break;
                case 2:
                    data = "Real World graph2";
                    break;
                case 3:
                    data = "Real World graph3";
                    break;
            }
            break;
    }

    string algorithm;
    switch(alg){
        case 1:
            algorithm = "Backtracking Algorithm";
            break;
        case 2:
            algorithm = "Triangular Inequality Heuristic";
            break;
        case 3:
            algorithm = "Christophides Heuristic";
            break;
        case 4:
            algorithm = "TSP in the Real World";
            break;
    }

    cout << "Using " << algorithm << " on " << data  << endl;
    cout << "Cost: " << calculatePathCost(g,path) << endl;
    cout << "Path: ";
    for (auto x : path){
        cout << x->getInfo() << " ";
    }
    cout << endl << "Execution time: " << duration << " milliseconds" << std::endl;


    file << "Using " << algorithm << " on " << data  << endl;
    file << "Cost: " << calculatePathCost(g,path) << endl;
    file << "Path: ";
    for (auto x : path){
        file << x->getInfo() << " ";
    }
    file << endl << "Execution time: " << duration << " milliseconds" << std::endl;
    file << "===========================================================================" << endl;
}

double Menu::calculatePathCost(Graph<int> &g, vector<Vertex<int>*> path){
    double currentCost = 0;
    auto first = g.findVertex((*path.begin())->getInfo());
    for (auto it = path.begin() + 1; it != path.end(); it++) {
        auto second = g.findVertex((*it)->getInfo());
        for (auto e : first->getAdj()) {
            if (e->getDest()->getInfo() == second->getInfo()) currentCost += e->getWeight();
        }
        first = second;
    }
    return currentCost;
}
