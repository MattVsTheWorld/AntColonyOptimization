#include "utils.h"
#include <sstream>
#include <fstream>
#include <iterator>
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>

/**
 * @file utils.h/cpp
 * @brief number generation, statistic formulas, distances, load and save to file.
 */

// CSV file parser
// Credit: https://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c
class CSVRow
{
public:
    std::string const& operator[](std::size_t index) const      {return m_data[index];}
    std::size_t size() const                                    {return m_data.size();}
    void readNextRow(std::istream& str)
    {
        std::string line;
        std::getline(str, line);

        std::stringstream lineStream(line);
        std::string cell;

        m_data.clear();
        while(std::getline(lineStream, cell, ','))
            m_data.push_back(cell);
        // This checks for a trailing comma with no Data after it.
        if (!lineStream && cell.empty())
            // If there was a trailing comma then add an empty element.
            m_data.emplace_back("");
    }
private:
    std::vector<std::string> m_data;
};

std::istream& operator>>(std::istream& str, CSVRow& data)
{
    data.readNextRow(str);
    return str;
}

class CSVIterator
{
public:
    typedef std::input_iterator_tag     iterator_category;
    typedef CSVRow                      value_type;
    typedef std::size_t                 difference_type;
    typedef CSVRow*                     pointer;
    typedef CSVRow&                     reference;

    explicit CSVIterator(std::istream& str)  :m_str(str.good()?&str:nullptr) { ++(*this); }
    CSVIterator()                   :m_str(nullptr) {}

    // Pre Increment
    CSVIterator& operator++()               {if (m_str) { if (!((*m_str) >> m_row)){m_str = nullptr;}}return *this;}
    // Post increment
    CSVIterator operator++(int)             {CSVIterator    tmp(*this);++(*this);return tmp;}
    CSVRow const& operator*()   const       {return m_row;}
    CSVRow const* operator->()  const       {return &m_row;}

    bool operator==(CSVIterator const& rhs) {return ((this == &rhs) || ((this->m_str == nullptr) && (rhs.m_str == nullptr)));}
    bool operator!=(CSVIterator const& rhs) {return !((*this) == rhs);}
private:
    std::istream*       m_str;
    CSVRow              m_row;
};

double Utils::manhattanDistance(std::pair<double, double> h1, std::pair<double, double> h2){
    /**
    * Calculate manhattan distance between two points
    * Manhattan distance: |x1 - x2| + |y1 - y2|
    * -- Parameters --
    * std::pair<double, double> h1  : first point
    * std::pair<double, double> h2  : second point
    * -- Return --
    * Calculated distance
    **/
    return abs(h1.first - h2.first) + abs(h1.second - h2.second);
}

double Utils::euclideanDistance(std::pair<double, double> h1, std::pair<double, double> h2){
    /**
    * Calculate euclidean distance between two points
    * Euclidean distance: √((x1 - x2)² + (y1 - y2)²)
    * -- Parameters --
    * std::pair<double, double> h1  : first point
    * std::pair<double, double> h2  : second point
    * -- Return --
    * Calculated distance
    **/
    return sqrt(pow(h1.first - h2.first, 2) + pow(h1.second - h2.second, 2));
}

double Utils::generateRngZeroOne(){
    /* Return a random number between 0 and 1 */
    return p(rng);
}

void Utils::saveCoordsToCSV(std::deque<std::pair<double, double>> &coordMap, int num_holes) {
    /**
    * Save coordinates of points to a CSV file
    * -- Parameters --
    * std::deque<std::pair<double,double>>& coordMat    : double ended queue containing coordinates of points
    **/
    // create an ofstream for the file output
    std::ofstream outputFile;
    std::string filename = "../Instances/Generated/coordinates_" + std::to_string(num_holes) +".csv";
    // create and open the .csv file
    outputFile.open(filename);
    // write Data to the file
    for (auto & i : coordMap)
        outputFile << i.first << "," << i.second << std::endl;
    // close the output file
    outputFile.close();
}

void Utils::loadFromCSV(std::deque<std::pair<double,double>>& coordMat, const std::string& filename){
    /**
    * Load coordinates of points from a CSV file
    * -- Parameters --
    * std::deque<std::pair<double,double>>& coordMat    : double ended queue containing coordinates of points
    * const std::string& filename                       : name of the file to load
    **/
    std::ifstream file(filename);
    for(CSVIterator loop(file); loop != CSVIterator(); ++loop)
        coordMat.emplace_back(std::make_pair(std::stod((*loop)[0]), std::stod((*loop)[1])));
}

void Utils::saveToDAT(doubleMap& times) {
    /**
    * Save distance matrix to a DAT file
    * -- Parameters --
    * doubleMap &times             : Matrix (vector of vectors) of distances/times to be filled
    **/
    unsigned long side = times.size();
    std::ofstream outputFile;
    std::ostringstream output;
    output << "../Instances/Generated/tsp" << side << ".dat";
    std::string filename = output.str();
    // create and open the file
    outputFile.open(filename);
    outputFile << side << '\n';
    for (int i = 0; i < side; i++) {
        for (int j = 0; j < side; j++) {
            std::ostringstream wr;
            (j >= i) ? wr << times[i][j] : wr << times[j][i];
            outputFile << wr.str();
            unsigned int tab = FORMAT - wr.str().size();
            for(int k=0; k < (tab > 1 ? tab : 2); k++)
                outputFile << " ";
        }
        outputFile << '\n';
    }
    outputFile.close();
}

long Utils::loadFromDAT(doubleMap &times, const std::string &filename) {
    /**
    * Load distance matrix from a DAT file
    * -- Parameters --
    * doubleMap &times                  : Matrix (vector of vectors) of distances/times to be filled
    * const std::string& filename       : name of the file to load
    **/
    int numHoles;
    std::ifstream in(filename);
    in >> numHoles;
    // --- Allocation ---
    times.resize(numHoles);
    for (int i = 0; i < numHoles; i++)
        times[i].resize(numHoles);
    // --- Generation ---
    for (int i = 0; i < numHoles; i++){
        // Iterate neighbor of holes
        for (int j = 0; j < numHoles; j++) {
            in >> times[i][j];
        }
    }
    return numHoles;
}

void Utils::stdev(const std::vector<double> &v, double& mean, double& stdev) {
    /*
     * Calculate the mean and standard deviation of a vector of doubles
     * Credits:
     * https://stackoverflow.com/questions/7616511/calculate-mean-and-standard-deviation-from-a-vector-of-samples-in-c-using-boos
     * */
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    mean = sum / v.size();
    std::vector<double> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(), [mean](double x) { return x - mean; });
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    stdev = std::sqrt(sq_sum / v.size());
}

double Utils::percentDifference(double approx, double best) {
    // Calculate the percent difference between two values
    return ((approx - best) / ((approx + best) / 2)) * 100;
}

Utils::Utils() {
    uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed>>32)}; // easy to use
    rng.seed(ss);
}

double Utils::isZero(double num) {
    // Consider very small values as zero
    if (num < 0.00001)
        return 0;
    else return num;
}

