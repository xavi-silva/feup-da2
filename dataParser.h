//
// Created by joaop on 03/05/2024.
//

#ifndef UNTITLED_DATAPARSER_H
#define UNTITLED_DATAPARSER_H

#include "dataStructures/Graph.h"

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Function declarations
void ToyGraphParser(const std::string& filename, Graph<int>& graph);
void ExtraFullyConnectedGraphsParser(const std::string& filename, Graph<int>& graph, int n);
void RealWorldGraphParser(const std::string& filename, Graph<int>& graph);

#endif //UNTITLED_DATAPARSER_H