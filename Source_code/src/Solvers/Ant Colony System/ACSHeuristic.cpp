#include "ACSHeuristic.h"
#include <cstdio>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <random>
#include <future>
#include <thread>

/**
 * @file ACSHeuristic.h/cpp
 * @brief Handles agents and global updates for ACS heuristic.
 */

void ACSHeuristic::init() {
    /**
    * Initialize the necessary variables
    * */
    pheromones.resize(numHoles);                    // Pheromone values
    deltaPheromones.resize(numHoles);               // Pheromone variation
    probs.resize(numHoles);                         // Probability vector <p, hole_index> (filled from a specific hole)
    bestRoute.resize(numHoles);                     // Best path so far
    for(int i=0; i<numHoles; i++){
        pheromones[i].resize(numHoles);
        deltaPheromones[i].resize(numHoles);
        probs[i] = std::make_pair(-1.0, -1.0);

        for (int j=0; j<numHoles; j++){
            pheromones[i][j]        = T_0;
            deltaPheromones[i][j]   = 0.0;
        }
    }
    // Ants
    routes.resize(numAnts);                         // Route found [ant_idx][hole_idx]
    antColony.resize(numAnts);                      // Vector of ants that will perform optimization
    for(int i=0; i<numAnts; i++){
        routes[i].resize(numHoles);
        antColony[i].init(i, numHoles, &bestLen, &bestRoute, &pheromones,
                                    &deltaPheromones, *distances, &ut, alpha,
                                    beta, local_evaporation_rate, q_0);
        for (int j=0; j<numHoles; j++)
            routes[i][j] = -1;
    }
}

void ACSHeuristic::reset() {
    /* Reset current values */
    pheromones.clear();
    deltaPheromones.clear();
    probs.clear();
    bestRoute.clear();
    routes.clear();
    antColony.clear();
    init();
}

void ACSHeuristic::printSpecifics(){
    /**
    * Print the specifics being run
    * */
    std::cout << "-- Parameters --" << std::endl;
    std::cout << "    Number of ants           :      " << numAnts << std::endl;
    std::cout << "    Number of holes          :      " << numHoles << std::endl;
    std::cout << "    Number of iterations     :      " << iterations << std::endl;
    std::cout << "    Concurrency enabled      :      " << ((concurrent) ? "Yes" : "No") << std::endl;
    std::cout << "    Misc parameters..." << "\n    | Alpha: " <<  alpha << " | Beta: "
                << beta << " | Local e.r. : " << local_evaporation_rate
                << " | Global e.r. : "  << global_evaporation_rate << " |\n    | Starting trail : "
                << T_0 << " | Pseudo-random acceptance : " << q_0 << " |" << std::endl;

}

void ACSHeuristic::printPheromones() {
    /**
    * Print the pheromone trail values between holes
    * */
    std::cout << "------------------------"
                 "\n--- Pheromone Values ---\n"
                 "------------------------"<< std::endl;
    std::cout << "  | ";
    for (int i = 0; i<numHoles; i++)
        printf("%5d   ", i);

    std::cout << std::endl << "- | ";
    for (int i = 0; i<numHoles; i++)
        std::cout << "---------";
    std::cout << std::endl;

    for(int i=0; i<numHoles; i++){
        std::cout << i << " | ";
        for(int j=0; j<numHoles; j++){
            if (i==j){
                printf("%5s   ", "~");
                continue;
            }
            printf("%7.3f ", pheromones[i][j]);
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

int ACSHeuristic::valid(int ant_k) {
    /**
    * Check if the route of an ant is valid
    * -- Parameters --
    * int ant_k: ant being taken into consideration
    * -- Return --
    * 0 if path is valid, one of for errors if not (in range [-1,-4])
    * */
    for (int i=0; i<numHoles-1; i++){
        int hole_i = routes[ant_k][i];
        int hole_j = routes[ant_k][i+1];
        // Index of hole is not valid (incomplete path)
        if (hole_i < 0 || hole_j < 0)
            return invalidHoleIndex;

        // One or more holes is repeated more than once
        for (int j=0; j<i-1; j++) {
            if (routes[ant_k][i] == routes[ant_k][j])
                return repeatedHoles;
        }
    }
    return noError;
}

void ACSHeuristic::printResults() {
    /**
    * Print results of optimization
    * */
    std::cout << "-- Optimization results --" << std::endl;
    std::cout << "    Best path: ";
    for (auto hole : bestRoute)
        std::cout << hole << " ";
    std::cout << "\n    Number of ants        :      " << numAnts << std::endl;
    std::cout << "    Number of iterations  :      " << iterations << std::endl;
    std::cout << "    Found obj             :      " << bestLen << std::endl;
    if (optimalSolution != -1) {
        std::cout << "    Optimal obj           :      " << optimalSolution << std::endl;
        std::cout << "    This is " << std::setprecision(4) << Utils::percentDifference(bestLen, optimalSolution)
                  << "% off the optimal solution." << std::endl;
    }
    std::cout << "    Optimization took "
              << solveTime << " milliseconds (" << solveTime/1000.0 << " second(s))."  << std::endl;
}

//void ACSHeuristic::localPheromoneUpdateAS() {
//    /*
//     * (!) LEGACY - PART OF AS, NOT USED IN ACS (!)
//     * Update the value of pheromones (once every iteration)
//     * */
//    // -- Generation of delta tau values --
//    // Iterate all ants
//    for (int ant_k=0; ant_k < numAnts; ant_k++){
//        double r_len = antColony[ant_k].pathWeight();
//        // Iterate edges used by the ant
//        for (int r=0; r<numHoles-1; r++){
//            int hole_i = routes[ant_k][r];
//            int hole_j = routes[ant_k][r+1];
//            // delta tau is 1/len of the entire tour, if the edge was visited
//            deltaPheromones[hole_i][hole_j] += 1 / r_len;
//            deltaPheromones[hole_j][hole_i] += 1 / r_len;
//        }
//    }
//    // -- Update phase of pheromones --
//    for (int i=0; i<numHoles; i++){
//        for (int j=0; j<numHoles; j++){
//            // Update rule = (1 - rho) * (tau_i_j) + delta_tau_i_j
//            pheromones[i][j] = (1 - local_evaporation_rate) * pheromones[i][j]
//                    +  deltaPheromones[i][j];
//            deltaPheromones[i][j] = 0.0;
//        }
//    }
//}

bool ACSHeuristic::isBest(int i, int j){
    /*
    * Check if two holes belong to the best route
    * -- Parameters --
    * int i :   outgoing hole
    * int j :   incoming hole
    * -- Return --
    * True if the edge i-j belongs to the current best tour, false otherwise
    */
    for (int x=0; x<numHoles-1; x++) {
        if (i == bestRoute[i] && j == bestRoute[i+1])
            return true;
    }
    return false;
}

void ACSHeuristic::globalPheromoneUpdate() {
    /*
     * Update pheromones globally, only on best edges.
     * The global update of pheromones is the reciprocal of the best length so far
     * */
    for(int i=0; i<numHoles; i++){
        for(int j=0; j<numHoles; j++){
            if (i==j)
                continue;
            // All pheromone trails decay
            pheromones[i][j] = (1 - global_evaporation_rate) * pheromones[i][j];
            pheromones[j][i] = (1 - global_evaporation_rate) * pheromones[j][i];
            // If the edge belongs to the best path, also increase the pheromone level
            if(isBest(i,j)){
                pheromones[i][j] += global_evaporation_rate * (1 / bestLen);
                pheromones[j][i] += global_evaporation_rate * (1 / bestLen);
            }
        }
    }

}

void ACSHeuristic::optimize() {
    /**
    * Optimize problem with a given number of iterations
    * */
    // Enable/Disable concurrency
    std::launch policy;
    if (concurrent)
        policy = std::launch::async;
    else
        policy = std::launch::deferred;
    std::random_device rng;
    std::mt19937 gen(rng());
    std::uniform_int_distribution<> startingDistribution(0, numHoles - 1);

    auto start = std::chrono::high_resolution_clock::now();
    // -- Optimization --
    for (int it=0; it<iterations; it++){
        // Send out each ant
        // --- FUTURES ---
        std::vector<std::future<void>> results;
        results.reserve(numAnts);
        for (int ant_k=0; ant_k < numAnts; ant_k++)
            results.emplace_back(std::async(policy, &Ant::execute, &antColony[ant_k],
                    startingDistribution(gen), &routes[ant_k]));
        for (auto & result : results)
            result.get();
        // --- THREADS ---
//        std::vector<std::thread> threads;
//        threads.reserve(numAnts);
//        for (int ant_k=0; ant_k < numAnts; ant_k++)
//            threads.emplace_back(&Ant::execute, &antColony[ant_k], startingDistribution(gen), &routes[ant_k]);
//
//        for (auto & thread : threads)
//            thread.join();
//        threads.clear();

        globalPheromoneUpdate();

        // Reset paths
        for (int k=0; k<numAnts; k++) {
            for (int j = 0; j < numHoles; j++)
                routes[k][j] = -1;
        }
    }
    //printPheromones();
    auto end = std::chrono::high_resolution_clock::now();
    solveTime = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
}