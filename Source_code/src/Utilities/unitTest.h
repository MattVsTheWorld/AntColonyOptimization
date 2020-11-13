#ifndef UNITTEST_H
#define UNITTEST_H

/**
 * @file unitTest.h/cpp
 * @brief run multiple tests and collect statistics.
 */

#include "../Solvers/solverManager.h"
#include "../Utilities/utils.h"
#include "../Solvers/Ant Colony System/ACSHeuristic.h"
#include <iomanip>

class unitTest {
public:
    unitTest();
    void runTests(char **argv, unsigned numTests, ACSparameters params, boardType type);
    void printResults();
    void saveResults();
private:
    // doubleMap loadedDistances;
    // The number of holes for each instance
    std::vector<int> numHoles;
    // Outer vector  : num_intervals number of different instances (with different number of holes)
    // Middle vector : n_problems_per_size times for each instance with same number of holes
    // Inner vector  : times to go between holes in specific instance
    std::vector<std::vector<doubleMap>> times;
    // Objective/Time values for all instances
    std::vector<std::vector<resultValues>> solutions;

};


#endif //UNITTEST_H
