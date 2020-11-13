#ifndef ANT_H
#define ANT_H

/**
 * @file Ant.h/cpp
 * @brief Ant agent for the ACS heuristic.
 */

#include "../../Utilities/typesAndDefs.h"
#include "../../Utilities/utils.h"
#define T_0 0.1

// Ant class. Parameters and functions are described in the .cpp file
class Ant {
public:
    Ant() : id(0), numHoles(0), startHole(0), bestLen(nullptr), route(nullptr), bestRoute(nullptr),
            pheromones(nullptr), deltaPheromones(nullptr), alpha(0.5), beta(0.5), local_evaporation_rate(0.1),  q_0(0.9), ut(nullptr){}

    void init(unsigned _id, int _n, double* _best, std::vector<int>* _bestR, doubleMap* _pher,
              doubleMap* _deltaPher, doubleMap& _dist, Utils* _u, double _a, double _b, double _ler, double _q);
    bool visited(int c);
    double pathWeight();
    double explorationProbability(int hole_i, int hole_j);
    int pickNextHole(int max);
    int explore(int currentHole);
    int exploit(int currentHole);
    void generateRoute();
    void localPheromoneUpdate(int idxSoFar);
    void execute(int _start, std::vector<int>* _route);
private:
    unsigned id;
    int numHoles, startHole;
    double* bestLen;
    std::vector<int> *route, *bestRoute;
    bool *visitedNodes;
    std::vector<std::pair<double, int>> probs;
    doubleMap *pheromones, *deltaPheromones;
    doubleMap distances;
    double alpha, beta, local_evaporation_rate, q_0;
    Utils *ut;
};


#endif // ANT_H
