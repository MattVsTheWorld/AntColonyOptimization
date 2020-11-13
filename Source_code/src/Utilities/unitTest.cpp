#include "unitTest.h"
#include <sstream>
#include <fstream>
/**
 * @file unitTest.h/cpp
 * @brief run multiple tests and collect statistics.
 */

unitTest::unitTest() = default;

void unitTest::runTests( char **argv, unsigned numTests, ACSparameters params, boardType type) {
    /**
    * Run various tests
    * -- Parameters --
    * int char **argv           : command line arguments (see main.cpp)
    * unsigned numTests         : number of times each test (on the same instance) should be run
    * ACSparameters params      : parameters for the ACS heuristic
    * boardType type            : type of boards to generate
    * */
    solverManager mng(&solutions, params, &numHoles, &times, SILENT,
                      std::atoi(argv[N_PROBLEMS_PER_SIZE]), std::atoi(argv[NUM_INTERVALS]), numTests);
    std::cout << "-- General specifics --" << std::endl;
    std::cout << "    Number of ants        :      " << params.numAnts << std::endl;
    std::cout << "    Number of iterations  :      " << params.iterations << std::endl;
    std::cout << "    Number of tests       :      " << numTests << std::endl;
    // --- Parameters ---
    mng.generate(std::atoi(argv[UPPER_BOUND]), type, EUCLIDEAN);
    // Cplex - Exact testing
//    mng.solveAllCPLEX();
    // ACS - Heuristic testing
//    mng.solveAllACS();
    // Solve both, collect statistics
    mng.solveAllWithStats(true);
}

void unitTest::printResults() {

    // Outer vector  : num_intervals number of different instances (with different number of holes)
    // Middle vector : n_problems_per_size times for each instance with same number of holes
    // Inner vector  : times to go between holes in specific instance

    // Each of these elements has a different number of holes
    for (int i=0; i<solutions.size(); i++) {
        // Each of these elements has the same number of holes, but is a different instance
        for (auto solution : solutions[i]){
            std::cout << "    Number of holes in this instance  :      " << numHoles[i] << std::endl;
            std::cout << "    Heuristic obj                     :      " << solution.approxObj << std::endl;
            std::cout << "    Optimal obj                       :      " << solution.optimalObj << std::endl;
            std::cout << "    This is " << std::setprecision(4) << Utils::percentDifference(solution.approxObj, solution.optimalObj)
                      << "% off the optimal solution." << std::endl;
            std::cout << "    Optimal optimization took (on average) "
                      << solution.optimalTime << " milliseconds (" << std::setprecision(4) <<  (solution.optimalTime)/1000.0 << " second(s))."  << std::endl;
            std::cout << "    Heuristic optimization took (on average) "
                      << solution.approxTime << " milliseconds (" << std::setprecision(4) <<  (solution.approxTime)/1000.0 << " second(s))."  << std::endl;

        }
    }
}

void unitTest::saveResults() {

    // --- Exact Solution ---
    std::ofstream outputFileExact;
    std::ostringstream output_exact;
    output_exact << "../Instances/Data/exact.txt";
    std::string filenameExact = output_exact.str();
    outputFileExact.open(filenameExact);
    // --- Approximate Solution ---
    std::ostringstream output_approx;
    std::ofstream outputFileApprox;
    output_approx << "../Instances/Data/approx.txt";
    std::string filenameApprox = output_approx.str();
    outputFileApprox.open(filenameApprox);
    outputFileExact  << "--- EXACT SOLUTIONS ---" << '\n';
    outputFileApprox  << "--- APPROX SOLUTIONS ---" << '\n';
    // Each i has different number of holes
    for(int i=0; i<solutions.size(); i++){
        outputFileExact  << numHoles[i] << '\n';
        outputFileApprox << numHoles[i] << '\n';
        outputFileExact  << "- Obj    time (ms) -" << '\n';
        outputFileApprox << "- Obj    time (ms) -" << '\n';
        // Each j has same number of holes, different configuration
        for (auto solution : solutions[i]){
            // Save numHoles[i] as x
            outputFileExact  << solution.optimalObj << "    " << solution.optimalTime << '\n';
            outputFileApprox << solution.approxObj << "    " << solution.approxTime << '\n';
        }
        outputFileExact  << "------------" << '\n';
        outputFileApprox << "------------" << '\n';
    }
    outputFileExact  << "EOF";
    outputFileApprox  << "EOF";
    outputFileExact.close();
    outputFileApprox.close();
}
