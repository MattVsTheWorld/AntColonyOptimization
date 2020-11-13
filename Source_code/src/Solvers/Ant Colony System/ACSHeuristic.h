#ifndef ACSHEURISTIC_H
#define ACSHEURISTIC_H

/**
 * @file ACSHeuristic.h/cpp
 * @brief Handles agents and global updates for ACS heuristic.
 */

#include "../../Utilities/typesAndDefs.h"
#include "../../Instance Generators/boardGenerator.h"
#include "../../Utilities/utils.h"
#include <deque>
#include <utility>
#include "Ant.h"
#include <climits>
// (unused) error values for invalid path found
enum validity { noError = 0, invalidHoleIndex = -1, repeatedHoles = -3 };

class ACSHeuristic {
public:
    /* Unused constructor for coordinate Matrix */ /*
    ACSHeuristic(ACSparameters params, std::deque<std::pair<double,double>>& coordMap, double _sol, bool _sync) :
            numAnts(params.numAnts), numHoles(coordMap.size()), iterations(params.iterations), alpha(params.alpha),
            beta(params.beta), local_evaporation_rate(params.rho), global_evaporation_rate(params.omega), q_0 (params.greediness),
            distMode(params.distMode),
            concurrent(_sync)
    {
        boardGenerator::generateTimes(*distances, coordMap, distMode);
        init();
    }
    */
    ACSHeuristic(ACSparameters params, doubleMap *times, double _sol, bool _sync) :
            numAnts(params.numAnts), numHoles(times->size()), iterations(params.iterations), alpha(params.alpha),
            beta(params.beta), local_evaporation_rate(params.rho), global_evaporation_rate(params.omega), q_0 (params.greediness),
            distMode(params.distMode), optimalSolution(_sol), concurrent(_sync)
    {
        distances = times;
        init();
    }

    void init();                            // Initialize structures
    void printPheromones();                 // Print pheromone trails
    void printResults();                    // Print results of optimization
    void printSpecifics();                  // Print specifics being used
    void optimize ();                       // Run Ant Colony System optimization
    long getTime() { return solveTime; }
    double getObj() { return bestLen; }
    void reset();
private:
    int valid(int ant_k);                   // Check whether a path is valid (not needed, feasible solutions are created)
    // void localPheromoneUpdateAS();       // Local pheromone update system for the (worse) Ant System algorithm
    void globalPheromoneUpdate();           // Global pheromone update, ran after every iteration on best path
    bool isBest(int i, int j);
    // Parameters and variables are better understood in their context in the .cpp file
    // --- Parameters ---
    int numAnts, numHoles, iterations;
    double alpha, beta, local_evaporation_rate, global_evaporation_rate, q_0;
    int distMode;
    double optimalSolution = -1;
    bool concurrent;
    // --- Variables ---
    double bestLen = static_cast<double>(INT_MAX);
    std::vector<int> bestRoute;
    intMap routes;
    doubleMap pheromones, deltaPheromones, *distances;
    std::vector<std::pair<double, int>> probs;
    Utils ut;
    long solveTime = -1;
    std::vector<Ant> antColony;
};

#endif //ACSHEURISTIC_H
