#ifndef TSPSOLVER_H
#define TSPSOLVER_H

/**
 * @file TSPSolver.h/cpp
 * @brief manages the CPLEX exact solution solver.
 */

#include "cpxmacro.h"
#include "../Utilities/typesAndDefs.h"

// --- Data container ---
class Data {
public:
    Data(int _numHoles, doubleMap _times): numHoles(_numHoles), timesMap(std::move(_times)) {}
    Data()= default;
    int numHoles{};
    doubleMap timesMap;
};

// --- Actual solver ---
class TSPSolver {
public:
    TSPSolver(Env _env, Prob _lp, Data _data)
            : env(_env), lp(_lp), data(std::move(_data)) {
        allocateMaps();
    }    // Constructor
    void initLP();                              // Initialize cplex LP with instance Data (vars and constraints)
    void solveLP(int mode);                     // Solve current LP
    void allocateMaps();                        // Initialize index maps
    int getSolveTime() { return solveTime; }
    double getObj() { return objVal; }
private:
    intMap map_x;
    intMap map_y;
    static const int NAME_SIZE = 512;
    char name[NAME_SIZE]{};

protected:
    long solveTime = -1;
    Env env;
    Prob lp;
    Data data;
    double objVal = static_cast<double>(INT_MAX);
};



#endif //TSPSOLVER_H
