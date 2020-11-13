#include "figureGenerator.h"
#include "../Utilities/utils.h"
#include <vector>
#include <random>
#include <utility>
#include <tuple>
#include <fstream>
//
/**
 * @file figureGenerator.h/cpp
 * @brief Handles generation of random polygons and eventually filler holes.
 */

void figureGenerator::addRandomFigure(std::deque<std::pair<double,double>>& coordMat, int  blockIdx,
                                      std::deque<block>& supportVec, int& generatedHoles, int availableHoles){
    /**
    * Add a random figure with a random value of vertices (between 3 and MAX_POLY_SIZE, or the number of available holes
    * if it exceeds MAX_POLY_SIZE)
    * -- Parameters --
    * std::deque<std::pair<double,double>>& coordMat        : double ended queue containing coordinates of points
    * int  blockIdx                                         : index of block where figure should be inserted
    * std::deque<block>& supportVec                         : Double ended queue containing the "block" structures
    * int& generatedHoles                                   : number of holes generated so far
    * int availableHoles                                    : holes still available (before reaching the requested amount)
    * */
    std::random_device rng;
    std::mt19937 gen(rng());
    // Restrict choices based on how many holes are available
    int max = (availableHoles <= MAX_POLY_SIZE) ? availableHoles : MAX_POLY_SIZE;

    std::uniform_int_distribution<> distr(3, max);
    int numVerts = distr(gen);
    std::uniform_int_distribution<> distr2(1, 90); // Generate a random inclination
    generatePolygon(numVerts, supportVec[blockIdx].side/POLY_RADIUS_RATIO, (distr2(gen)* M_PI) / 180,
            coordMat, supportVec[blockIdx]);
    generatedHoles+=numVerts;

}

void figureGenerator::generatePolygon(int numVertices, double radius, double theta,
        std::deque<std::pair<double,double>>& coordMat, block& currentBlock) {
    /**
    * Insert a polygon with given number of vertices, radius and orientation, inside the current block.
    * It's coordinates are added to the "coordMat" dequeue.
    * -- Parameters --
    * int numVertices                                   : number of vertices the polygon should have
    * double radius                                     : the radius the polygon should have
    * double theta                                      : the angle of orientation of the polygon
    * std::deque<std::pair<double,double>>& coordMat    : double ended queue containing coordinates of points
    * block& currentBlock                               : block where the polygon should be inserted
    * */
    currentBlock.numVerts = numVertices;
    for (int i = 0; i < numVertices; i++)
        coordMat.emplace_back(std::make_pair(radius * cos(2 * M_PI * i / numVertices + theta) + currentBlock.midCoords.first,
                                radius * sin(2 * M_PI * i / numVertices + theta) + currentBlock.midCoords.second));
}


void figureGenerator::addFillerHoles(std::deque<std::pair<double,double>>& coordMat, int num){
    /**
    * Add "filler holes" to the board (1 or 2, since at least 3 are require for a polygon)
    * -- Parameters --
    * std::deque<std::pair<double,double>>& coordMat    : double ended queue containing coordinates of points
    * int num                                           : number of filler points to add
    * */
    try{
        std::pair<double,double> coord;
        Utils ut;
        switch(num) {
            case 1:
                // Add a hole in the middle
                coord.first = SIDE / 2;
                coord.second = SIDE / 2;
                coordMat.emplace_back(coord);
                break;
            case 2:
                if (ut.generateRngZeroOne() > 0.5){
                    // Add holes in a "diagonal"
                    coord.first = SIDE / 4;
                    coord.second = SIDE / 4;
                    coordMat.emplace_back(coord);
                    coord.first = SIDE * (3.0 / 4);
                    coord.second = SIDE * (3.0 / 4);
                    coordMat.emplace_back(coord);
                } else {
                    // Flipped diagonal
                    coord.first = SIDE / 4;
                    coord.second = SIDE * (3.0 / 4);
                    coordMat.emplace_back(coord);
                    coord.first = SIDE * (3.0 / 4);
                    coord.second = SIDE / 4;
                    coordMat.emplace_back(coord);
                }
                break;
            default:
                throw std::runtime_error(std::string(__FILE__) + ":\n "
                                         + "Invalid usage; only 1 or 2 filler holes may be added.");
        }
    } catch(std::exception& e)
    {
        std::cout << ">>>EXCEPTION: " << e.what() << std::endl;
    }
}

