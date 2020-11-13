#include "solverManager.h"
#include "../Instance Generators/boardGenerator.h"
#include "../Utilities/utils.h"
#include "Ant Colony System/ACSHeuristic.h"
#include <numeric>
#include <iomanip>
#include <sstream>
#include <fstream>

/**
 * @file solverManager.h/cpp
 * @brief Given a set of instances, solves them and saves statistics to file/prints them.
 */

// --- PRIVATE ---
long solverManager::solveIndividualCPLEX(const Data& data, int inst_num, resultValues& solutions) {
    /**
    * Solve an individual problem with given Data with the CPLEX exact solution
    * -- Parameters --
    * const Data& Data      : number of holes and distance matrix of the problem
    * int inst_num          : number of the instance being run (if more than one)
    * double& objVal        : variable where to save the exact solution to the problem
    * -- Return --
    * The amount of time (ms) required to solve the instance
    * */
    if (informativeness != SILENT) {
        std::cout << "    - Test # " << inst_num + 1 << std::endl;
        std::cout << "    Solving an instance with " << data.numHoles << " holes." << std::endl;
    }
    // --- Statistics ---
    double minObj = static_cast<double>(INT_MAX), maxObj = 0;
    double minTime = static_cast<double>(INT_MAX), maxTime = 0;
    std::vector<double> objectives;
    std::vector<double> timings;
    // ------------------
    // if numTests > 1, the same test is repeated and the average time is returned
    for(int i=0; i<numTests; i++) {
        DECL_ENV(env);
        DECL_PROB(env, lp);
        TSPSolver tsp(env, lp, data);
        tsp.initLP();
        tsp.solveLP(informativeness);
        auto obj = tsp.getObj();
        auto time = tsp.getSolveTime();
        objectives.emplace_back(obj);
        timings.emplace_back(time);
        if (obj > maxObj)
            maxObj = obj;
        if (obj < minObj)
            minObj = obj;
        if (time > maxTime)
            maxTime = time;
        if (time < minTime)
            minTime = time;
    }
    double meanObj, stdevObj;
    Utils::stdev(objectives, meanObj, stdevObj);
    solutions.optimalObj = meanObj;
    double meanTime, stdevTime;
    Utils::stdev(timings, meanTime, stdevTime);
    solutions.optimalTime = meanTime;
    CPLEXstatistics.emplace_back(
            stats{meanTime, stdevTime, maxTime, minTime,
                  meanObj, stdevObj, maxObj, minObj});
    return std::ceil(solutions.optimalTime);
}

long solverManager::solveIndividualACS(const Data& data, int inst_num, resultValues& solutions){
    /**
    * Solve an individual problem with given Data with the ACS metaheuristic
    * -- Parameters --
    * const Data& Data              : number of holes and distance matrix of the problem
    * int inst_num                  : number of the instance being run (if more than one)
    * const double optimalSolution  : the optimal solution to the problem (not strictly necessary; comparison-related)
    * -- Return --
    * The amount of time (ms) required to solve the instance
    * */
    if (informativeness != SILENT) {
        std::cout << "    - Test # " << inst_num + 1 << std::endl;
        std::cout << "    Solving an instance with " << data.numHoles << " holes." << std::endl;
    }
    // --- Statistics ---
    double minObj = static_cast<double>(INT_MAX), maxObj = 0;
    double minTime = static_cast<double>(INT_MAX), maxTime = 0;
    std::vector<double> objectives;
    std::vector<double> timings;
    // ------------------
    auto distances = data.timesMap;
    ACSHeuristic ants(params, &distances, solutions.optimalObj, true);
    for(int i=0; i<numTests; i++) {
        ants.optimize();
        auto obj = ants.getObj();
        auto time = ants.getTime();
        objectives.emplace_back(obj);
        timings.emplace_back(time);
        if (obj > maxObj)
            maxObj = obj;
        if (obj < minObj)
            minObj = obj;
        if (time > maxTime)
            maxTime = time;
        if (time < minTime)
            minTime = time;
        ants.reset();
    }
    double meanObj, stdevObj;
    Utils::stdev(objectives, meanObj, stdevObj);
    solutions.approxObj = meanObj;
    double meanTime, stdevTime;
    Utils::stdev(timings, meanTime, stdevTime);
    solutions.approxTime = meanTime;
    ACSstatistics.emplace_back(
            stats{meanTime, stdevTime, maxTime, minTime,
                  meanObj, stdevObj, maxObj, minObj});
    // if numTests > 1, the same test is repeated and the average time is returned
    return std::ceil(solutions.approxTime);
}

// --- PUBLIC ---
void solverManager::generate(int maxHoles, boardType type, int dist_type){
    /**
    * Generate the required instances of the problem (see parameters description in solverManager.h)
    * -- Parameters --
    * int maxHoles          : number of holes of the biggest instance created
    * boardType type        : type of board (cointoss, polygons, random...)
    * int dist_type         : distance function to use in the generation of the distances matrix
    * */
    try {
        if (num_intervals >= maxHoles)
            throw std::runtime_error(std::string(__FILE__) + ":\n"
                                     + "Number of intervals has to be smaller than upper bound!");
        std::cout << "---------------------------------------" << std::endl;
        std::cout << "    Generating " << n_problems_per_size
                  << " problems for each instance (hole numbers)." << std::endl;
        std::cout << "    Generating " << num_intervals << " hole numbers, ranging between 0 (skipped, no holes) and "
                  << maxHoles << " (max number of holes)." << std::endl;

        // Generate the hole numbers, and how many travel times will be needed
        for (int i = 0; i < num_intervals; i++) {
            unsigned int num = (i + 1) * maxHoles / num_intervals;
            num_holes->push_back(num);
        }

        // There will be num_intervals different number of holes
        // (e.g. 10 instances that differ in how many holes the board has)
        {
            // For each instance with different number of holes
            for (int i = 0; i < num_intervals; i++) {
                // Evaluate different configuration for the same number of holes
                // (e.g. 5 instances that have same number of holes but different times)
                // For each instance with same number of holes
                for (int j = 0; j < n_problems_per_size; j++) {

                    if (informativeness!=SILENT)
                        std::cout << "Asking for a board with " << (*num_holes)[i] << " holes..." << std::endl;
                    switch(type){
                        case RandomAsym:
                            boardGenerator::generateAsymBoard((*times)[i][j], (*num_holes)[i], MAX_DIAG);
                            break;
                        case RandomSym:
                            boardGenerator::generateSymBoard((*times)[i][j], (*num_holes)[i], MAX_DIAG);
                            break;
                        case CointossGrid:
                            boardGenerator::generateCoinTossGridBoard((*times)[i][j], (*num_holes)[i], dist_type, informativeness);
                            break;
                        case Polygons:
                            boardGenerator::generateGeometricBoard((*times)[i][j], (*num_holes)[i], dist_type);
                            break;
                    }
                }
            }

        }
    } catch(std::exception& e)
    {
        std::cout << ">>>EXCEPTION: " << e.what() << std::endl;
    }
}

void solverManager::solveAllCPLEX() {
    /**
    * Solve all instances using the CPLEX exact method
    * */
    // Solve each instance
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "    CPLEX: Initiating solve sequence..." << std::endl;
    // Iterate number of holes
    for (int i = 0; i < num_intervals; i++) {
        if (informativeness != SILENT)
            std::cout << "    >>> INSTANCE [" << i+1 << "] (" << (*num_holes)[i] << " holes)" << std::endl;
        // Iterate different tests for same holes number
        double avg_time = 0;
        for (int j = 0; j < n_problems_per_size; j++) {
            avg_time += solveIndividualCPLEX(Data((*num_holes)[i], (*times)[i][j]), j, (*results)[i][j]);
        }
        if (informativeness != SILENT)
            std::cout << "    (!) Avg time (rounded up): " << std::ceil(avg_time/n_problems_per_size) << "ms ("
                  << std::ceil(avg_time/n_problems_per_size)/1000.0 << " second(s))."  << std::endl;
    }
    std::cout << "    Solved all instances." << std::endl;
    std::cout << "---------------------------------------" << std::endl;
}

void solverManager::solveAllACS(){
    /**
    * Solve all instances using the ACS metaheuristic
    * */
    // Solve each instance
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "    ACS: Initiating solve sequence..." << std::endl;
    // Iterate number of holes
    for (int i = 0; i < num_intervals; i++) {
        if (informativeness != SILENT)
            std::cout << "    >>> INSTANCE [" << i+1 << "] (" << (*num_holes)[i] << " holes)" << std::endl;
        // Iterate different tests for same holes number
        double avg_time = 0;
        for (int j = 0; j < n_problems_per_size; j++)
            avg_time += solveIndividualACS(Data((*num_holes)[i], (*times)[i][j]), j, (*results)[i][j]);
        if (informativeness != SILENT)
            std::cout << "    (!) Avg time (rounded up): " << std::ceil(avg_time/n_problems_per_size) << "ms ("
                  << std::ceil(avg_time/n_problems_per_size)/1000.0 << " second(s))."  << std::endl;
    }
    std::cout << "    Solved all instances." << std::endl;
    std::cout << "---------------------------------------" << std::endl;

}

void solverManager::solveAllWithStats(bool save) {
    /**
    * Solve all instances using both methods, collect statistics
    * */
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "    Initiating solve sequence..." << std::endl;
    // Iterate number of holes
    for (int i = 0; i < num_intervals; i++) {
        // Iterate different tests for same holes number
        std::cout << "Solving instances with " << (*num_holes)[i] << " holes..." << std::endl;
        for (int j = 0; j < n_problems_per_size; j++) {
            solveIndividualCPLEX(Data((*num_holes)[i], (*times)[i][j]), j, (*results)[i][j]);
            solveIndividualACS(Data((*num_holes)[i], (*times)[i][j]), j, (*results)[i][j]);
        }
        // Recap CPLEX and ACS stats
        std::cout << "    Collecting statistics on " << n_problems_per_size
            << " instances with " << (*num_holes)[i] << " holes ("<< numTests <<" tests for each instance)..." << std::endl;
        double minObj = static_cast<double>(INT_MAX), maxObj = 0;
        double minTimeCPLEX = static_cast<double>(INT_MAX), maxTimeCPLEX = 0;
        double minTimeACS= static_cast<double>(INT_MAX), maxTimeACS = 0;
        double minError = static_cast<double>(INT_MAX), maxError = 0;
        std::vector<double> stdevTimeCPLEX, stdevTimeACS, stdevObjACS;
        std::vector<double> timingsACS, timingsCPLEX;
        std::vector<double> errors;
        /*
         * Each instance is run "numTests" times
         * Statistics are collected on the averages/extremes of these runs for each number of holes
         * */
        for(int idx=0; idx<n_problems_per_size; idx++){
            // --- CPLEX ---
            timingsCPLEX.emplace_back(CPLEXstatistics[idx].meanTime_ms);
            stdevTimeCPLEX.emplace_back(CPLEXstatistics[idx].stdev_ms);
            if (CPLEXstatistics[idx].maxTime > maxTimeCPLEX)    { maxTimeCPLEX = CPLEXstatistics[idx].maxTime; }
            if (CPLEXstatistics[idx].minTime < minTimeCPLEX)    { minTimeCPLEX = CPLEXstatistics[idx].minTime; }
            // --- ACS ---
            timingsACS.emplace_back(ACSstatistics[idx].meanTime_ms);
            stdevTimeACS.emplace_back(ACSstatistics[idx].stdev_ms);
            stdevObjACS.emplace_back(ACSstatistics[idx].stdev_obj);
            if (ACSstatistics[idx].maxTime > maxTimeACS)    { maxTimeACS = ACSstatistics[idx].maxTime; }
            if (ACSstatistics[idx].minTime < minTimeACS)    { minTimeACS = ACSstatistics[idx].minTime; }
            if (ACSstatistics[idx].maxObj > maxObj)         { maxObj = ACSstatistics[idx].maxObj; }
            if (ACSstatistics[idx].minObj < minObj)         { minObj = ACSstatistics[idx].minObj; }
            double error = Utils::percentDifference(ACSstatistics[idx].meanObj, CPLEXstatistics[idx].meanObj);
            if (error > maxError) { maxError = error; }
            if (error < minError) { minError = error; }
            errors.emplace_back(error);
        }
        double meanTimeCPLEX_ms, stdevMeanCPLEX_ms;
        Utils::stdev(timingsCPLEX, meanTimeCPLEX_ms, stdevMeanCPLEX_ms);
        double meanTimeACS_ms, stdevMeanACS_ms;
        Utils::stdev(timingsACS, meanTimeACS_ms, stdevMeanACS_ms);
        double meanError, stdevError;
        Utils::stdev(errors, meanError, stdevError);
        // SUM OF STUFF
        plotStatistics.emplace_back(plotStats{(*num_holes)[i],
                                              meanTimeCPLEX_ms, stdevMeanCPLEX_ms, maxTimeCPLEX, minTimeCPLEX,
                                              meanTimeACS_ms, stdevMeanACS_ms, maxTimeACS, minTimeACS,
                                              (std::accumulate(stdevObjACS.begin(), stdevObjACS.end(),0.0))/num_intervals,
                                              meanError, stdevError, maxError, minError});
        // clear
        CPLEXstatistics.clear();
        ACSstatistics.clear();
    }
    printStatistics();
    if (save)
        saveStatistics();
    std::cout << "    Solved all instances." << std::endl;
    std::cout << "---------------------------------------" << std::endl;
}

void solverManager::printStatistics() {
    /**
    * Print statistics
    * */
    for (auto stats : plotStatistics){
        std::cout << "---------------------------------" << std::endl;
        std::cout << "Number of holes in the instance: " << stats.num_holes  << std::endl;
        std::cout << "-- Solution quality statistics --" << std::endl;
        std::cout << "      Mean error      : " << std::setprecision(4) << Utils::isZero(stats.meanError)  << "%" << std::endl;
        std::cout << "      Error stdev     : " << std::setprecision(4) << Utils::isZero(stats.stdevError) << "%" << std::endl;
        std::cout << "      Max error       : " << std::setprecision(4) << Utils::isZero(stats.maxError)  << "%" << std::endl;
        std::cout << "      Min error       : " << std::setprecision(4) << Utils::isZero(stats.minError) << "%" << std::endl;
        std::cout << "      The ACS heuristic has a mean stdev on objective value of " << Utils::isZero(stats.stdevACSObj) << std::endl;
        std::cout << "-- CPLEX time performance --"  << std::endl;
        std::cout << "      Mean time       : " << std::setprecision(4) << stats.meanTimeCPLEX_ms/1000 << "s" << std::endl;
        std::cout << "      Time stdev      : " << std::setprecision(4) << stats.stdevMeanCPLEX_ms/1000 << "s" << std::endl;
        std::cout << "      Max time        : " << std::setprecision(4) << stats.maxTimeCPLEX/1000 << "s" << std::endl;
        std::cout << "      Min time        : " << std::setprecision(4) << stats.minTimeCPLEX/1000 << "s" << std::endl;
        std::cout << "-- ACS time performance --"  << std::endl;
        std::cout << "      Mean time       : " << std::setprecision(4) << stats.meanTimeACS_ms/1000 << "s" << std::endl;
        std::cout << "      Time stdev      : " << std::setprecision(4) << stats.stdevMeanACS_ms/1000 << "s" << std::endl;
        std::cout << "      Max time        : " << std::setprecision(4) << stats.maxTimeACS/1000 << "s" << std::endl;
        std::cout << "      Min time        : " << std::setprecision(4) << stats.minTimeACS/1000 << "s" << std::endl;
        std::cout << "---------------------------------" << std::endl;
    }
}

void solverManager::saveStatistics() {
    /**
    * Save statistics
    * */
    std::ofstream outputFile;
    std::ostringstream output;
    output << "../Instances/Data/statistics.csv";
    std::string filenameExact = output.str();
    outputFile.open(filenameExact);
    outputFile << "num_holes,mean error,stdev error,max error,min error,stdev obj,"
                  "mean time CPLEX,stdev time CPLEX, max time CPLEX,min time CPLEX,"
                  "mean time ACS,stdev time ACS, max time ACS,min time ACS\n";
    for (auto stats : plotStatistics){

        outputFile << stats.num_holes  << ",";
        outputFile << std::setprecision(4) << Utils::isZero(stats.meanError)  << ",";
        outputFile << std::setprecision(4) << Utils::isZero(stats.stdevError) << ",";
        outputFile << std::setprecision(4) << Utils::isZero(stats.maxError) << ",";
        outputFile << std::setprecision(4) << Utils::isZero(stats.minError) << ",";
        outputFile << Utils::isZero(stats.stdevACSObj) << ",";

        outputFile << std::setprecision(4) << stats.meanTimeCPLEX_ms/1000 << ",";
        outputFile << std::setprecision(4) << stats.stdevMeanCPLEX_ms/1000 << ",";
        outputFile << std::setprecision(4) << stats.maxTimeCPLEX/1000 << ",";
        outputFile << std::setprecision(4) << stats.minTimeCPLEX/1000 << ",";
        outputFile << std::setprecision(4) << stats.meanTimeACS_ms/1000 << ",";
        outputFile << std::setprecision(4) << stats.stdevMeanACS_ms/1000 << ",";
        outputFile << std::setprecision(4) << stats.maxTimeACS/1000 << ",";
        outputFile << std::setprecision(4) << stats.minTimeACS/1000 << "\n";
    }
    outputFile << "EOF";
    outputFile.close();
}
