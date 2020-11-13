#ifndef UTILS_H
#define UTILS_H

/**
 * @file utils.h/cpp
 * @brief number generation, statistic formulas, distances, load and save to file.
 */

#include <deque>
#include "typesAndDefs.h"
#include <random>
#include <chrono>

class Utils {
public:

    Utils();

    static void stdev(const std::vector<double> &vec, double& mean, double& stdev);
    static double isZero(double num);
    static double percentDifference(double approx, double best);
    static double manhattanDistance(std::pair<double, double> h1, std::pair<double, double> h2);
    static double euclideanDistance(std::pair<double, double> h1, std::pair<double, double> h2);
    double generateRngZeroOne();
    // Load and save coordinates to a CSV file
    static void saveCoordsToCSV(std::deque<std::pair<double, double>> &coordMap, int num_holes);
    static void loadFromCSV(std::deque<std::pair<double,double>>& coordMap, const std::string& filename);
    // Load and save distance matrix from a DAT file
    static long loadFromDAT(doubleMap& times, const std::string& filename);
    static void saveToDAT(doubleMap& times);

private:
    // random real number generator
    std::mt19937_64 rng;
    std::uniform_real_distribution<double> p;
};


#endif //UTILS_H
