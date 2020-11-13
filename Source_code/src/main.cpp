#include "Solvers/solverManager.h"
#include "Utilities/unitTest.h"
#include <climits>

/**
 * @file main.cpp
 * @brief Runs program, handles arguments.
 */


int status;
char errmsg[BUF_SIZE];

int main(int argc, char** argv) {
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "Program starting..." << std::endl;
    std::cout << "---------------------------------------" << std::endl;
    ACSparameters params = {40, 800, 1.0,
                            4.0, .8, .1,
                            .95, EUCLIDEAN};
    try {
        if (argc > NUM_PARAMS || argc == 1)
            throw std::runtime_error(std::string(__FILE__) + ": " + "\nIncorrect usage of parameters!");

    } catch(std::exception& e)
    {
        std::cout << ">>>EXCEPTION: " << e.what() << std::endl;
        std::cout << "Usage [1] (string): "
                     "\n(1) Path of '.dat' file to open."
                  << std::endl;
        std::cout << "Usage [2] (int values): "
                     "\n(1) Number of problems for each size."
                     "\n(2) Number of intervals to test between 0 and the upper bound."
                     "\n(3) Upper bound."
                     "\n(4) Number of tests to run (on same instance, average is taken)."
                  << std::endl;
        return 0;
    }
    if(argc == 2) {
        // -----------------------------------
        // ----- Test specific instances -----
        doubleMap times;
        Utils::loadFromDAT(times, argv[FILENAME]);
        int test_pool = 10;
        double best = -1;
        std::vector<double> results;
        std::vector<double> timings;
        double minTime = static_cast<double>(INT_MAX), maxTime = 0;
        double mean, stdev;
        for(int i=0; i<test_pool; i++) {
            DECL_ENV(env);
            DECL_PROB(env, lp);
            TSPSolver tsp(env, lp, Data(times.size(), times));
            tsp.initLP();
            tsp.solveLP(SILENT);
            best = tsp.getObj();
            double time = tsp.getSolveTime();
            timings.emplace_back(time);
            if (time > maxTime)
                maxTime = time;
            if (time < minTime)
                minTime = time;
        }
        Utils::stdev(timings, mean, stdev);
        std::cout << "-- CPLEX --" << std::endl;
        std::cout << "Avg CPLEX time (s) : " << std::setprecision(4) << mean/1000 << std::endl;
        std::cout << "Max (time)         : " << (maxTime/1000) << "\nMin (time)         : " << (minTime/1000) << std::endl;
        std::cout << "Stdev (time, s)    : " << std::setprecision(4) << (stdev/1000) << std::endl;
        std::cout << "=========================" << std::endl;
        std::cout << "BEST solution      : " << best << std::endl;
        std::cout << "---------------------------------------" << std::endl;
        timings.clear();
        double minObj = static_cast<double>(INT_MAX), maxObj = 0;
        minTime = static_cast<double>(INT_MAX), maxTime= 0;
        for(int i=0; i<test_pool; i++) {
            ACSHeuristic ants(params, &times, best, true);
            ants.optimize();
            auto obj = ants.getObj();
            auto time = ants.getTime();
            results.emplace_back(obj);
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
        std::cout << "-- ACS --" << std::endl;
        Utils::stdev(timings, mean, stdev);
        std::cout << "Avg Time (s)       : " << mean/1000 << std::endl;
        std::cout << "Max (time, s)      : " << (maxTime/1000) << "\nMin (time, s)      : " << (minTime/1000) << std::endl;
        std::cout << "Stdev (time, s)    : " << std::setprecision(4) << (stdev/1000) << std::endl;
        Utils::stdev(results, mean, stdev);
        std::cout << "=========================" << std::endl;
        std::cout << "Avg Obj            : " << std::setprecision(4) << mean << std::endl;
        std::cout << "Max (obj)          : " << maxObj << "\nMin (obj)          : " << minObj << std::endl;
        std::cout << "Stdev (obj)        : " << std::setprecision(4) << stdev << std::endl;
        std::cout << "Solution is " << std::setprecision(4) << Utils::percentDifference(mean, best)
                  << "% off the optimal solution." << std::endl;

    } else {
        // -----------------------------------
        // -- Test various generated boards --
        unitTest unit;
        unit.runTests(argv, std::atoi(argv[NUM_TESTS]), params, CointossGrid);
        //unit.printResults();
        unit.saveResults();
    }
    // -----------------------------------
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "Programing ending..." << std::endl;
    std::cout << "---------------------------------------" << std::endl;
    return 0;
}