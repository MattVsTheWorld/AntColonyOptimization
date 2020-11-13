#ifndef BOARDGENERATOR_H
#define BOARDGENERATOR_H
#include "figureGenerator.h"

/**
 * @file boardGenerator.h/cpp
 * @brief Handles generation of boards of various kinds.
 */

class boardGenerator {
private:
    static void findHolesPerSide(int& numHoles, int& holesPerSide, int informativeness);
    static void randomGrid(doubleMap& times, int holesPerSide, double interval, int originalNum, int distMode, int informativeness );
    static void splitBoard(std::deque<block>& supportVec, std::pair<double, double> c, double side);
    static int findBlock(std::deque<block>& supportVec, std::deque<std::pair<double, double>>& coordMat, int& generated);
    static void deleteFirstBlock(std::deque<block>& supportVec, std::deque<std::pair<double, double>>& coordMat, int &generated);
public:
    boardGenerator()= default;
    static double findDistance(int distMode, std::pair<double, double> h1, std::pair<double, double> h2);
    // Generates random times between holes. Times are asymmetric.
    static void generateAsymBoard(doubleMap& times, int numHoles, double ub);
    // Generate random board. Times are random, but symmetric (NOTE: might not satisfy triangle inequality!).
    static void generateSymBoard(doubleMap& times, int numHoles, double ub);
    // Generate a board with holes on a grid. Holes appear with p=0.5.
    static void generateCoinTossGridBoard(doubleMap& times, int numHoles, int distMode, int informativeness);
    // Generated holes at random on the board, as vertices of geometric figures (no intersection).
    static void generateGeometricBoard(doubleMap& times, int numHoles, int distMode);
    // Transform a deque of coordinates into a matrix of times
    static void generateTimes(doubleMap &times, std::deque<std::pair<double,double>>& coordMat,  int distMode);
};


#endif //BOARDGENERATOR_H
