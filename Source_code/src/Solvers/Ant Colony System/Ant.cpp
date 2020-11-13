#include "Ant.h"
#include <string>

/**
 * @file Ant.h/cpp
 * @brief Ant agent for the ACS heuristic.
 */

void Ant::init(unsigned _id, int _n, double* _best, std::vector<int>* _bestR, doubleMap* _pher,
          doubleMap* _deltaPher, doubleMap& _dist, Utils* _u, double _a, double _b, double _ler, double _q)
    {
    /**
    * Initialize parameters for the current Ant
    * */
    id = _id;                               // ID of current Ant/process
    numHoles = _n;                          // Number of holes of the instance
    bestLen = _best;                        // Best objective function (so far)
    bestRoute = _bestR;                     // Best route found (so far)
    pheromones = _pher;                     // Pheromone trail matrix
    deltaPheromones = _deltaPher;           // Pheromone variation matrix
    distances = _dist;                      // Distances matrix
    alpha = _a;                             // Importance of pheromone value
    beta = _b;                              // Importance of heuristic value
    local_evaporation_rate = _ler;          // Local evaporation rate of pheromones
    q_0 = _q;                               // Pseudo random acceptance (ratio of acceptance of greedy steps)
    ut = _u ;                               // Utility random generator
    probs.resize(numHoles);
    for(int i=0; i<numHoles; i++)
        probs[i] = std::make_pair(-1.0, -1.0);

    visitedNodes = new bool[numHoles];      // Truth vector of whether a hole has been  visited or not
    for(int i=0; i<numHoles; i++)
        visitedNodes[i] = false;
}

bool Ant::visited(int c) {
    /*
     * Determine whether ant_k has visited hole with index c
     * -- Parameters --
     * int c    : index of hole being checked
     * -- Return --
     * True if the hole with index c has been visited, false otherwise
     * */
    return visitedNodes[c];
}

double Ant::pathWeight() {
    /**
    * Find the path weight (objective function value) of the path found by ant_k
    * -- Return --
    * Sum of the weights of the edges traversed by the ant in its path
    * */
    double sum = 0.0;
    int i;
    for (i=0; i<numHoles-1; i++) {
        sum += distances[(*route)[i]][(*route)[i + 1]];
    }
    sum += distances[(*route)[i]][(*route)[0]];
    return sum;
}

double Ant::explorationProbability(int hole_i, int hole_j) {
    /**
     * Probabilistic formula to determine the probability value of an ant to move between two given holes
     * -- Parameters --
     * int hole_i: index of outgoing hole
     * int hole_j: index of incoming hole
     * -- Return --
     * The probability of an ant in hole_i to move to hole_j
     * */
    // Tau value for edge i-j. Corresponds to the pheromone trail edge
    auto tau_i_j = static_cast<double>(pow((*pheromones)[hole_i][hole_j], alpha));
    // Eta is a heuristic function
    auto eta_i_j = static_cast<double>(pow(1/ distances[hole_i][hole_j], beta));
    // Alpha: importance of pheromone trail. Beta: Importance of heuristic function
    double sum_weights = 0.0;
    for (int c=0; c < numHoles; c++){
        // Set of non-visited holes
        if (!visited(c)){
            auto eta = static_cast<double>(pow (1/distances[hole_i][c], beta));
            auto tau = static_cast<double>(pow ((*pheromones)[hole_i][c], alpha));
            sum_weights += eta * tau;
        }
    }
    return (eta_i_j * tau_i_j) / sum_weights;
}

int Ant::pickNextHole(int max) {
    /**
    * Determine next hole to walk to, randomly chosen based on probabilities
    * -- Return --
    * Index of chosen hole
    * */
    // Generate a random uniform probability
    double p = ut->generateRngZeroOne();
    int i = 0;
    double sum = probs[i].first;
    // Determine stochastically (based on p) which city to go to
    while (sum < p && i < max-1){
        i++;
        sum += probs[i].first;
    }
    // return the selected hole to move to
    return probs[i].second;
}

int Ant::explore(int currentHole) {
    /**
    * Decide a new hole based on an "exploration" strategy (probabilistic based on pheromones)
    * -- Parameters --
    * int currentHole   : hole where the current Ant is residing
    * -- Return --
    * Index of chosen hole
    * */
    int count = 0;
    try{

        // Iterate over other holes
        // -- Generation phase: generate probabilities --
        for (int j=0; j<numHoles; j++){
            // if same hole, skip
            if (currentHole == j)
                continue;
            // If the hole j has NOT been visited by ant_k, add to the pool of possibly chosen
            if (!visited(j)) {
                // Probability to move to j, based on pheromone values and parameters
                probs[count].first = explorationProbability(currentHole, j);
                // Remember that the associated probability refers to j
                probs[count].second = j;
                count++;
            }
        }
        if (count == 0)
            throw std::runtime_error(std::string(__FILE__) + ": " + "Deadlocked ant! (no hole can be selected)");
    } catch(std::exception& e) {
        std::cout << ">>>EXCEPTION: " << e.what() << std::endl;
        return -1;
    }
    // --- Selection phase: chose a hole to move to ---
    return pickNextHole(count);
}

int Ant::exploit(int currentHole) {
    /**
    * Decide a new hole based on a deterministic "exploitation" strategy.
    * Edges with highest pheromones are always chosen.
    * -- Parameters --
    * int currentHole   : hole where the current Ant is residing
    * -- Return --
    * Index of chosen hole
    * */
    int best_idx = -1;
    double best_tau_eta = 0.0;
    // Iterate over other holes
    for (int j=0; j<numHoles; j++){
        // if same hole, skip
        if (currentHole == j)
            continue;
        if (!visited(j)) {
            // Deterministic value, whose maximum should be chosen in exploitation
            auto current_tau_eta = static_cast<double>(pow((*pheromones)[currentHole][j], alpha))
                                   *  static_cast<double>(pow(1/ distances[currentHole][j], beta));
            if (current_tau_eta > best_tau_eta){
                best_tau_eta = current_tau_eta;
                best_idx = j;
            }
        }
    }
    // --- Selection phase: chose a hole to move to ---
    return best_idx;
}


void Ant::localPheromoneUpdate(int idxSoFar) {
    /*
     * Update the value of pheromones (each time an edge is selected)
     * -- Parameters --
     * int idxSoFar         : last element of the route vector that has been filled
     * */
        (*pheromones)[(*route)[idxSoFar]][(*route)[idxSoFar+1]] = (1 - local_evaporation_rate) * (*pheromones)[(*route)[idxSoFar]][(*route)[idxSoFar+1]]
                                        + local_evaporation_rate * T_0;
        (*pheromones)[(*route)[idxSoFar+1]][(*route)[idxSoFar]] = (1 - local_evaporation_rate) * (*pheromones)[(*route)[idxSoFar+1]][(*route)[idxSoFar]]
                                        + local_evaporation_rate * T_0;
}

void Ant::generateRoute() {
    /**
    * Generate a route for ant_k, based on exploration and exploitation strategies
    * */
    // Start in a random position
    (*route)[0] = startHole;
    for (int i=0; i<numHoles-1; i++) {
        // -- State transition rules: exploration/exploitation --
        auto p = ut->generateRngZeroOne();
        (*route)[i+1] = (p <= q_0) ? exploit((*route)[i]) : explore((*route)[i]);
        // Update visited list
        visitedNodes[(*route)[i+1]] = true;
        localPheromoneUpdate(i);
    }
}


void Ant::execute(int _start, std::vector<int>* _route){
    /**
    * Execute actions for current Ant (generate a route)
    * -- Parameters --
    * int _start                : starting hole for current Ant
    * std::vector<int>* _route  : Pointer to route object to be filled
    * */
    // Copy pointer to route in global routes vector
    route = _route;
    // Reset the ant
    for (int j=0; j < numHoles; j++) {
        (*route)[j] = -1;
        visitedNodes[j] = false;
    }
    // Randomly selected starting point
    startHole = _start;
    visitedNodes[_start] = true;
    // Generate the cycle for the current ant
    generateRoute();
    // Update the best length if it's better
    double r_len = pathWeight();
    if(r_len < (*bestLen)) {
        (*bestLen) = r_len;
        // Update best path
        for (int i = 0; i < numHoles; i++)
            (*bestRoute)[i] = (*route)[i];
    }
}