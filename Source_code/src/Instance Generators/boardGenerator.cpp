#include "boardGenerator.h"
#include "../Utilities/utils.h"
#include <random>
#include <chrono>

/**
 * @file boardGenerator.h/cpp
 * @brief Handles generation of boards of various kinds.
 */

// times == c_i_j
void boardGenerator::generateAsymBoard(doubleMap& times, int numHoles, double ub) {
    /**
    * Generate an instance with random weights for edges. Edges are asymmetric.
    * (a->b) != (b->a). Mostly for debugging, as the test are ran on symmetric TSP instances.
    * -- Parameters --
    * doubleMap& times      : Matrix (vector of vectors) of distances/times to be filled
    * int numHoles          : number of cities/holes to be generated
    * double ub             : the upper bound on the random value to be generated (minimum is 1.0)
     * */
    times.resize(numHoles);
    std::random_device rng;
    std::mt19937 gen(rng());
    std::uniform_real_distribution<> distribution(1.0, ub);
    for (int i = 0; i < numHoles; i++) {
        times[i].resize(numHoles);
        for (int j = 0; j < numHoles; j++){
            if (i == j)
                times[i][j] = 0.0;
            else {
                double t = distribution(gen);
                times[i][j] = t;
            }
        }
    }
}

void boardGenerator::generateSymBoard(doubleMap& times, int numHoles, double ub) {
    /**
    * Generate an instance with random weights for edges. Edges are symmetric.
    * (a->b) == (b->a) for all edges.
    * -- Parameters --
    * doubleMap& times      : Matrix (vector of vectors) of distances/times to be filled
    * int numHoles          : number of cities/holes to be generated
    * double ub             : the upper bound on the random value to be generated (minimum is 1.0)
     * */
    times.resize(numHoles);
    for(int i = 0; i < numHoles; i++)
        times[i].resize(numHoles);
    std::random_device rng;
    std::mt19937 gen(rng());
    std::uniform_real_distribution<> distribution(1.0, ub);
    for (int i = 0; i < numHoles; i++) {
        for (int j = i; j < numHoles; j++){
            if (i == j)
                times[i][j] = 0.0;
            else {
                double t = distribution(gen);
                times[i][j] = t;
                times[j][i] = t;
            }
        }
    }
}

void boardGenerator::findHolesPerSide(int& numHoles, int& holesPerSide, int informativeness) {
    /**
    * Find how many holes can fit in the side of the current board.
    * (uses the fact that the board is a square).
    * NOTE: only used in the generation of the cointoss board
    * -- Parameters --
    * int& numHoles         : number of holes.
    * int& holesPerSide     : calculated number of holes per side in the cointoss grid
    * int informativeness   : specifies how informative output messages should be
    * */
    double _tempHoles = sqrt(numHoles);
    if (informativeness!= SILENT && !((int) _tempHoles * (int) _tempHoles == numHoles ||
       ((int) _tempHoles+1) * ((int) _tempHoles+1) == numHoles)) { // in case of an off-by-one float error
        std::cout << "Unable to create grid with desired number of holes (" << numHoles
                  << "). Rounding number to closest square (";
    }
    holesPerSide = (int) std::round(_tempHoles);
    numHoles = holesPerSide*holesPerSide;
    if (informativeness!= SILENT)
        std::cout << numHoles << ") ..." << std::endl;
}

double boardGenerator::findDistance(int distMode, std::pair<double, double> h1, std::pair<double, double> h2){
    /**
    * Find distance between two coordinate points
    * -- Parameters --
    * int distMode                  : distance function to use
    * std::pair<double, double> h1  : first point
    * std::pair<double, double> h2  : second point
    * -- Return --
    * A double representing the distance between the two requested points
    * */
    try {
        switch (distMode) {
            case MANHATTAN:
                return Utils::manhattanDistance(h1, h2);
            case EUCLIDEAN:
                return Utils::euclideanDistance(h1, h2);
            default:
                throw std::runtime_error(std::string(__FILE__) + ": "
                                         + "Unrecognized distance identifier.");
        }
    } catch(std::exception& e)
    {
        std::cout << ">>>EXCEPTION: " << e.what() << std::endl;
        return -1;
    }
}

void boardGenerator::generateTimes(doubleMap &times, std::deque<std::pair<double,double>>& coordMat,  int distMode){
    /**
    * Generate a matrix (vector of vectors) of times (aka distances) given a list of coordinates representing
    * holes on the board. Generated distances are stored in the times doubleMap.
    * -- Parameters --
    * doubleMap &times                                  : Matrix (vector of vectors) of distances/times to be filled
    * std::deque<std::pair<double,double>>& coordMat    : double ended queue containing coordinates of points
    * int distMode                                      : distance function to use
    * */
    // --- Allocation ---
    times.resize(coordMat.size());
    for (int i = 0; i < coordMat.size(); i++)
        times[i].resize(coordMat.size());
    // --- Generation ---
    for (int i = 0; i < coordMat.size(); i++) {
        // Iterate neighbor of holes
        for (int j = i; j < coordMat.size(); j++) {
            // This will not be selected regardless at solve time
            if (i==j)
                times[i][j] = 0.0;
            else {
                double t = findDistance(distMode, coordMat[i], coordMat[j]);
                times[i][j] = t;
                times[j][i] = t;
            }
        }
    }
    // Comment this line to remove saving of instances
    Utils::saveToDAT(times);
}

void boardGenerator::randomGrid(doubleMap &times, int holesPerSide, double interval, int originalNum, int distMode, int informativeness) {
    /**
    * Generate a random grid, where holes are punched with p=0.5 until "originalNum" holes are created.
    * Could be vastly more efficient...
    * -- Parameters --
    * doubleMap &times          : Matrix (vector of vectors) of distances/times to be filled
    * int holesPerSide          : calculated number of holes per side in the cointoss grid
    * double interval           : minimum horizontal/vertical distance between holes
    * int originalNum           : exact number of holes that should appear
    * int distMode              : distance function to use
    * int informativeness       : specifies how informative output messages should be

    * */
    // Linearized "matrix"
    std::deque<std::pair<double,double>> coordMat;
    // --- Toss a coin to decide whether hole should appear or not ---
    std::mt19937_64 rng;
    // initialize the random number generator with time-dependent seed
    uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed>>32)};
    rng.seed(ss);
    // initialize a uniform distribution between 0 and 1
    std::uniform_real_distribution<double> p(0, 1);
    // --- Generate evenly spread coordinates on a grid ---
    // Keep generating until correct size
    while(coordMat.size() != originalNum) {
        for (int i = 0; i < holesPerSide; i++) {
            for (int j = 0; j < holesPerSide; j++) {
                // Simulates going through row/columns and picking whether the hole is made
                if (p(rng) > 0.5) {
                    std::pair<double, double> coord;
                    coord.first = j * interval + interval / 2;
                    coord.second = i * interval + interval / 2;
                    coordMat.push_back(coord);
                }
            }
        }
        if (coordMat.size() != originalNum)
            coordMat.clear();
    }

    // Comment this line to remove saving of coordinates
    Utils::saveCoordsToCSV(coordMat, originalNum);

    if (informativeness != SILENT)
        std::cout << "Generated a board with: " << coordMat.size() << " holes." << std::endl;
    // --- Populate distance map ---

    generateTimes(times, coordMat, distMode);
}



void boardGenerator::generateCoinTossGridBoard(doubleMap& times, int numHoles, int distMode, int informativeness) {
    /**
    * Generate the parameters and fill the cointoss grid board.
    * A "cointoss" grid is a board where numHoles appear at regular intervals. To make it so the board
    * is slightly different each time, some holes are skipped (a coin is tossed to decide whether it appears or not)
    * -- Parameters --
    * doubleMap &times          : Matrix (vector of vectors) of distances/times to be filled
    * int numHoles              : exact number of holes that should appear
    * int distMode              : distance function to use
    * int informativeness       : specifies how informative output messages should be

    * */
    try {
        // --- "Gridify" ---
        // Since holes appear with p=0.5, expected number of holes is preserved by doubling max
          int originalNum = numHoles;
        numHoles*=2;
        int holesPerSide;
        findHolesPerSide(numHoles, holesPerSide, informativeness);
        // With side 20, the upper bound on the number of holes is 400'000
        double interval = (SIDE / holesPerSide);
        // std::cout << holesPerSide << " " << SIDE << " " << interval << std::endl;
        if (interval < 0.01)
            throw std::runtime_error(std::string(__FILE__) + ":\n "
                                     + "Too many holes! Please try a smaller number");

        // --- Generation of times ---
        randomGrid(times, holesPerSide, interval, originalNum, distMode, informativeness);


    } catch(std::exception& e)
    {
        std::cout << ">>>EXCEPTION: " << e.what() << std::endl;
    }
}

void boardGenerator::splitBoard(std::deque<block>& supportVec, std::pair<double, double> c, double side){
    /**
    * Split a block of the board of a given center "c" into four smaller blocks
    * -- Parameters --
    * std::deque<block>& supportVec         : Double ended queue containing the "block" structures
    *  std::pair<double, double> c          : coordinates of the center of the block to split
    *  double side                          : side of the block to split
    * */
    side/=2;
    supportVec.emplace_back(block(true, std::make_pair(c.first-(side*.5), c.second-(side*.5)),  side, 0));
    supportVec.emplace_back(block(true, std::make_pair(c.first+(side*.5), c.second-(side*.5)),  side, 0));
    supportVec.emplace_back(block(true, std::make_pair(c.first-(side*.5), c.second+(side*.5)),  side, 0));
    supportVec.emplace_back(block(true, std::make_pair(c.first+(side*.5), c.second+(side*.5)),  side, 0));
}

void boardGenerator::deleteFirstBlock(std::deque<block>& supportVec, std::deque<std::pair<double, double>>& coordMat, int &generated){
    /**
    * Delete the first block of the queue
    * -- Parameters --
    * std::deque<block>& supportVec                     : Double ended queue containing the "block" structures
    * std::deque<std::pair<double, double>>& coordMat   : Double ended queue of point coordinates
    * int& generated                                    : number of holes generated so far
    * */
    generated-= supportVec.begin()->numVerts;
    // Delete coordinates - We always delete the "first" block (oldest added). This preserves the correspondence.
    coordMat.erase(coordMat.begin(),coordMat.begin()+supportVec.begin()->numVerts);
    // Delete first block
    supportVec.erase(supportVec.begin());
}
int boardGenerator::findBlock(std::deque<block>& supportVec, std::deque<std::pair<double, double>>& coordMat, int& generated){
    /**
    * Find a free block where a new polygon may be placed. If no free blocks are present, free one and split it
    * into 4, the return the index of the first new free block.
    * -- Parameters --
    * std::deque<block>& supportVec                     : Double ended queue containing the "block" structures
    * std::deque<std::pair<double, double>>& coordMat   : Double ended queue of point coordinates
    * int& generated                                    : number of holes generated so far
    * -- Return --
    * Index of free block
    * */
    bool notFound = false;
    int idx = 0;
    while(!notFound) {
        // If free
        if (supportVec[idx].clear) {
            supportVec[idx].clear = false;    // Now occupied
            return idx;                       // Retrieve index for coords
        } else {    // Keep searching
            idx++;
        }
        // No free block was found
        if(idx == supportVec.size())
            notFound = true;
    }
    // No free block found; delete one and split
    // Delete first, split in 4. Put last 4 in the back. Always delete first.
    splitBoard(supportVec, supportVec[0].midCoords,supportVec[0].side);
    // Delete first element
    deleteFirstBlock(supportVec, coordMat, generated);
    // Added 4 elements: idx is free. Removed 1 element: move back 1.
    supportVec[idx-1].clear = false;    // Now occupied
    return idx-1;                       // Retrieve index for coords
}

void boardGenerator::generateGeometricBoard(doubleMap& times, int numHoles, int distMode){
    /**
    * Generate a "geometric board", aka a board filled with holes in regular polygon shapes.
    * -- Parameters --
    * doubleMap &times          : Matrix (vector of vectors) of distances/times to be filled
    * int numHoles              : exact number of holes that should appear
    * int distMode              : distance function to use
    * */
    try {
        // --- Initialization ---
        int generatedHoles = 0;
        if (numHoles < 3)
            throw std::runtime_error(std::string(__FILE__) + ":\n "
                                     + "Invalid number of holes; at least 3 holes required for geometric generation.");
        std::deque<std::pair<double, double>> coordMat;
        std::deque<block> supportVec;
        // Initial split into 4 squares
        splitBoard(supportVec, std::make_pair(SIDE/2, SIDE/2), SIDE);

        // --- Generation; while still holes to punch ---
        do {
            int available = numHoles - generatedHoles;
            // --- Action decided based on how many holes remain to be punched
            switch(available){
                case 1:
                    figureGenerator::addFillerHoles(coordMat, 1);
                    generatedHoles+=1;
                    break;
                case 2:
                    figureGenerator::addFillerHoles(coordMat, 2);
                    generatedHoles+=2;
                    break;
                default:
                    // --- Select the first free, largest block (or split one to create free space) ---
                    figureGenerator::addRandomFigure(coordMat, findBlock(supportVec, coordMat, generatedHoles),
                            supportVec, generatedHoles, available);
                    break;
            }
        }  while(generatedHoles != numHoles);
        // Comment this line to remove saving of coordinates
        Utils::saveCoordsToCSV(coordMat, numHoles);
        generateTimes(times, coordMat, distMode);

    } catch(std::exception& e)
    {
        std::cout << ">>>EXCEPTION: " << e.what() << std::endl;
}

}
