#ifndef FIGUREGENERATOR_H
#define FIGUREGENERATOR_H

/**
 * @file figureGenerator.h/cpp
 * @brief Handles generation of random polygons and eventually filler holes.
 */

#include <iostream>
#include <deque>
#include "../Utilities/typesAndDefs.h"

// Simple structure to represent a block (small square area of the grid)
class block{
public:
    block(bool _c, std::pair<double, double> _coords, double _s, int _n)
            : clear(_c), midCoords(std::move(_coords)), side(_s), numVerts(_n)  {}
    bool clear;
    std::pair<double, double> midCoords;
    double side;
    int numVerts;
};

class figureGenerator {
public:
    static void generatePolygon(int numVertices, double radius, double theta,
                std::deque<std::pair<double,double>>& coordMat, block& currentBlock);
    static void addRandomFigure(std::deque<std::pair<double,double>>& coordMat, int blockIdx,
                                std::deque<block>& supportVec, int& generatedHoles, int availableHoles);
    static void addFillerHoles(std::deque<std::pair<double,double>>& coordMat, int num);

};


#endif //FIGUREGENERATOR_H
