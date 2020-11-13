#include "TSPSolver.h"
#include <iostream>
#include <chrono>

/**
 * @file TSPSolver.h/cpp
 * @brief manages the CPLEX exact solution solver.
 */

void TSPSolver::allocateMaps() {
    /* Allocate map for x and y vars */
    map_x.resize(data.numHoles);
    map_y.resize(data.numHoles);
    for (int i = 0; i < data.numHoles; i++){
        map_x[i].resize(data.numHoles);
        map_y[i].resize(data.numHoles);
        for(int j = 0; j < data.numHoles; j++) {
            map_x[i][j] = -1;
            map_y[i][j] = -1;
        }
    }
}

void TSPSolver::initLP() {
    /* Initialize the linear programming problem
     * -- Sets --
     * N (nodes) : e.g. i, j
     * A (arcs)  : binary relation on nodes (i,j)
     *
     * -- Parameters --
     * c_i_j: cost to move from i to j
     * 0    : starting node
     *
     * -- Decision variables --
     * x_i_j: flow shipped from i to j
     * y_i_j: (binary) whether i ships anything to j
     */

    // -----------------------
    // --- SETUP VARIABLES ---
    // Add x_i_j variables
    const int x_init = 0;
    int current_var_position = x_init;
    for (int i = 0; i < data.numHoles; i++)
    {
        // We skip x_i_0; by optimality, they will always be 0
        for (int j = 1; j < data.numHoles; j++)
        {
            if (i == j)
                continue;
            char x_type = 'C';
            double lb = 0.0;
            double ub = CPX_INFBOUND;
            double obj = 0.0; // x is not present in obj func
            snprintf(name, NAME_SIZE, "x_%i_%i", i, j);
            char *x_name = (char *) (&name[0]);
            CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &obj, &lb, &ub, &x_type, &x_name);
            // Add index to map
            map_x[i][j] = current_var_position;
            ++current_var_position;
        }

    }
    // Add y_i_j variables
    for (int i = 0; i < data.numHoles; i++)
    {
        // We don't remove y_i_0 variables
        for (int j = 0; j < data.numHoles; j++)
        {
            if (i == j)
                continue;
            char y_type = 'B';
            double lb = 0.0;
            double ub = 1.0;
            double obj = data.timesMap[i][j];
            snprintf(name, NAME_SIZE, "y_%i_%i", i, j);

            char *y_name = (char *) (&name[0]);
            CHECKED_CPX_CALL(CPXnewcols, env, lp, 1, &obj, &lb, &ub, &y_type, &y_name);
            // Add index to map
            map_y[i][j] = current_var_position;
            ++current_var_position;
        }
    }
    // -----------------------
    // --- ADD CONSTRAINTS ---
    // (1) Unit flow constraint
    // For all k... We skip node 0 (k belongs to Data.numHoles \ {0})
    // for all k
    //      sum { x_i_k } over i s.t. (i,k) exists
    //      - sum { x_k_j } over j s.t. (k,j) exists, j != 0
    for (int k = 1; k < data.numHoles; k ++){
        // For each k, there are Data.numHoles-1 vars x_i_k and Data.numHoles-2 vars x_k_j
        const int sizeOfVec = (data.numHoles - 1) + (data.numHoles - 2);
        std::vector<int> idx(sizeOfVec);
        std::vector<double> coef(sizeOfVec, 1.0);
        int current_index = 0;
        // x_i_k
        for (int i = 0; i < data.numHoles; i ++){
            if (i == k)
                continue;
            idx[current_index] = map_x[i][k];
            //coef[current_index] = 1.0;
            current_index ++;
        }
        // x_k_j
        for (int j = 1; j < data.numHoles; j ++){
            if (j == k)
                continue;
            idx[current_index] = map_x[k][j];
            coef[current_index] = -1.0;
            current_index ++;
        }
        double rhs = 1.0;
        char sense = 'E';
        int matbeg = 0;
        CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense,
                         &matbeg, &idx[0], &coef[0], nullptr, nullptr);
    }

    // (2) Sum of binary flows constraint
    // Personal note: only one is active, but may transfer any amount of flow (as long as net is 1)
    // for all i, sum { y_i_j } over j s.t. (i,j) arc exists = 1                (only one i_j arc is active)
    for(int i = 0; i < data.numHoles; i++){
        // Each MAY flow to the other Data.numHoles-1 nodes
        std::vector<int> idx(data.numHoles - 1);
        std::vector<double> coef(data.numHoles - 1, 1.0);
        char sense = 'E';
        int index = 0;
        for (int j = 0; j < data.numHoles; j++){
            if(i == j)
                continue;
            idx[index] = map_y[i][j];
            index++;
        }

        double rhs = 1.0;
        int matbeg = 0;
        CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense,
                         &matbeg, &idx[0], &coef[0], nullptr, nullptr);
    }

    // (3) Dual binary flow constraint
    // for all j, sum { y_i_j } over i s.t. (i,j) arc exists = 1
    for(int j = 0; j < data.numHoles; j++){
        // Each MAY flow to the other Data.numHoles-1 nodes
        std::vector<int> idx(data.numHoles - 1);
        std::vector<double> coef(data.numHoles - 1, 1.0);
        char sense = 'E';
        int index = 0;
        for (int i = 0; i < data.numHoles; i++) {
            if(i == j)
                continue;
            idx[index] = map_y[i][j];
            index++;
        }

        double rhs = 1.0;
        int matbeg = 0;
        CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense,
                         &matbeg, &idx[0], &coef[0], nullptr, nullptr);
    }

    // (4) Flow quantity constraint
    // for all arcs (i,j), j != 0, x_i_j <= (|Data.numHoles| - 1) * y_i_j
    // Becomes: x_i_j - (|Data.numHoles| - 1) * y_i_j <= 0
    // In other words, the amount shipped from any i to j is never greater than the max unit flow
    // If there is no ship from i to j, this has to be 0 (due to binary y_i_j)
    for(int i = 0; i < data.numHoles; i++){
        for (int j = 1; j < data.numHoles; j++) {
            if (i == j)
                continue;

            std::vector<int> idx(2);
            idx[0] = map_x[i][j];                           // x var
            idx[1] = map_y[i][j];                           // y var
            std::vector<double> coef(2);
            coef[0] = 1.0;                                  // for x
            coef[1] = - (data.numHoles - 1);                // for y
            char sense = 'L';
            double rhs = 0.0;
            int matbeg = 0;
            CHECKED_CPX_CALL(CPXaddrows, env, lp, 0, 1, idx.size(), &rhs, &sense,
                             &matbeg, &idx[0], &coef[0], nullptr, nullptr);
        }
    }

    // Write lp file (debug) // disabled for performance
    CHECKED_CPX_CALL(CPXwriteprob, env, lp, "Lab1.lp", nullptr);
}

void TSPSolver::solveLP(int mode) {
    /**
    * Solve linear programming problem
    * -- Parameters --
    * int mode  : level of informativeness
    * */

    try
    {
        // We expect the number of variables to be equal to the number of edges (Data.numHoles*(Data.numHoles-1)), twice
        // e.g. both x_1_2 and x_2_1
        // We have such number of x variables (amount shipped) and also y (binary, whether it ships)
        // We removed all x_i_0 vars; these correspond to the Data.numHoles - 1 edges we don't consider by optimality.
        int numVar = 2 * data.numHoles * (data.numHoles - 1) - (data.numHoles - 1);
        // Setup LP and functions
        // Optimize
        auto start = std::chrono::high_resolution_clock::now();
        CHECKED_CPX_CALL(CPXmipopt, env, lp);
        auto end = std::chrono::high_resolution_clock::now();
        solveTime = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
        if (mode!=SILENT)
            std::cout << "    Optimization took "
                      << solveTime << " milliseconds (" << solveTime/1000.0 << " second(s))."  << std::endl;

        // Print resulting objective value
        CHECKED_CPX_CALL(CPXgetobjval, env, lp, &objVal);
        if (mode!=SILENT)
            std::cout << "(!) Objective value: " << objVal << std::endl;

        // Check if everything is ok (variables number is good)
        int n = CPXgetnumcols(env, lp);
        if (n != numVar)
            throw std::runtime_error(std::string(__FILE__) + ": " + "different number of variables!" );
        else if (mode!=SILENT)
            std::cout << "    Correct number of variables." << std::endl;

        // Print variable solutions
        if (mode==VERBOSE) {
            std::vector<double> varVals;
            varVals.resize(n);
            CHECKED_CPX_CALL(CPXgetx, env, lp, &varVals[0], 0, n - 1);
            for (int i = 0; i < n; ++i)
                std::cout << "      var in position " << i << " : " << varVals[i] << std::endl;
        }

        // Save solution
        CHECKED_CPX_CALL(CPXsolwrite, env, lp, "Lab1.sol");

        // Free
        CPXfreeprob(env, &lp);
        CPXcloseCPLEX(&env);
    }
    catch(std::exception& e)
    {
        std::cout << ">>>EXCEPTION: " << e.what() << std::endl;
    }

}