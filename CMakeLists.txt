cmake_minimum_required(VERSION 3.27)
project(untitled)

set(CMAKE_CXX_STANDARD 17)

add_executable(untitled main.cpp
        dataParser.h
        dataParser.cpp
        menu.h
        dataStructures/Graph.h
        functions.cpp
        haversine.cpp
        haversine.h
)
