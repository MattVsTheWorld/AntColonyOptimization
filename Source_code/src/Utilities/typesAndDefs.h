#ifndef TYPESANDDEFS_H
#define TYPESANDDEFS_H

/**
 * @file typesAndDefs.h
 * @brief  Various types and macro definitions, useful to enhance readability
 */

#include <iostream>
#include <vector>
// Used for index maps
typedef std::vector<std::vector<int>> intMap;
// Used for "distance" maps
typedef std::vector<std::vector<double>> doubleMap;

// Parameters
struct ACSparameters{
    int numAnts;
    int iterations;
    double alpha;
    double beta;
    double rho;
    double omega;
    double greediness;
    int distMode;
};

// Assume arbitrary 200x200 board. Assume a max diagonal of ~280 (200*sqrt(2))
// NOTE: there's a minimum distance of 1.0
#define SIDE 200.0
#define MAX_DIAG SIDE*sqrt(2)
#define M_PI 3.14159265358979323846
#define MAX_POLY_SIZE 8
#define POLY_RADIUS_RATIO 3
#define FORMAT 8

// Silent: only print average times
#define SILENT 0
// Descriptive: print individual test times and obj functions
#define DESCRIPTIVE 1
// Verbose: print all solutions for all variables (excessively long)
#define VERBOSE 2

// INDEXES
#define FILENAME 1
#define N_PROBLEMS_PER_SIZE 1
#define NUM_INTERVALS 2
#define UPPER_BOUND 3
#define NUM_TESTS 4
#define NUM_PARAMS 5

// DISTANCES
#define MANHATTAN 0
#define EUCLIDEAN 1


#endif //TYPESANDDEFS_H
