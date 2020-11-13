#ifndef UTILSPROVIDER_H
#define UTILSPROVIDER_H

/**
 * @file solverManager.h/cpp
 * @brief Given a set of instances, solves them and saves statistics to file/prints them.
 */

#include "cpxmacro.h"
#include "TSPSolver.h"

// Types of board
enum boardType{ RandomAsym, RandomSym, CointossGrid, Polygons };
struct resultValues {
    double optimalObj;
    double optimalTime;
    double approxObj;
    double approxTime;
};

struct stats{
    // --- TIME statistics ---
    double meanTime_ms, stdev_ms, maxTime, minTime;
    // --- OBJ statistics ---
    double meanObj, stdev_obj, maxObj, minObj;
};

struct plotStats{
    int num_holes;
    // --- TIME statistics ---
    double meanTimeCPLEX_ms, stdevMeanCPLEX_ms, maxTimeCPLEX, minTimeCPLEX;
    double meanTimeACS_ms, stdevMeanACS_ms, maxTimeACS, minTimeACS;
    // --- OBJ/ERROR statistics ---
    double stdevACSObj;
    double meanError, stdevError, maxError, minError;
};

class solverManager {
private:
    long solveIndividualCPLEX(const Data& data, int inst_num, resultValues& solutions);
    long solveIndividualACS(const Data& data, int inst_num, resultValues& solutions);
    // --- Params ---
    std::vector<std::vector<resultValues>>* results;        // Structure containing objective/time values found in all instances
    ACSparameters params;                                   // Parameters for the ACS heuristic
    std::vector<int>* num_holes;                            // The number of holes of the various instances
    std::vector<std::vector<doubleMap>>* times;             // Structure containing ALL distance matrices of various instances
    int informativeness;                                    // Level of output messages
    int n_problems_per_size;                                // How many problems are generated for each size (e.g. same # holes)
    int num_intervals;                                      // How many intervals are generated between the 0 and the max number of holes
    unsigned numTests;                                      // How many time to run on the SAME instance
    std::vector<stats> CPLEXstatistics;                     // Useful statistics
    std::vector<stats> ACSstatistics;
    std::vector<plotStats> plotStatistics;
    void init(){
        results->resize(num_intervals);
        times->resize(num_intervals);
        num_holes->reserve(num_intervals);
        for (int i=0; i<num_intervals; i++) {
            (*results)[i].resize(n_problems_per_size);
            (*times)[i].resize(n_problems_per_size);
        }
    }
public:
    solverManager(std::vector<std::vector<resultValues>>* _objValues, ACSparameters _params, std::vector<int> * _num_holes,
                  std::vector<std::vector<doubleMap>> * _times, int _informativeness, int _n_problems_per_size, int _num_intervals,
                  unsigned _numTests) : results(_objValues), params(_params), num_holes(_num_holes), times(_times), informativeness(_informativeness),
                                        n_problems_per_size(_n_problems_per_size), num_intervals(_num_intervals), numTests(_numTests){
        init();
    }
    void generate(int maxHoles, boardType type, int dist_type);
    void solveAllCPLEX();
    void solveAllACS();
    void solveAllWithStats(bool save);
    void printStatistics();
    void saveStatistics();
};

#endif //UTILSPROVIDER_H
