#ifndef MINI_AI_CUP_3_SOLVER_H
#define MINI_AI_CUP_3_SOLVER_H

#include <chrono>
#include <array>

#include "s_olution.h"


using namespace std::chrono;

class Solver {
private:
    std::array<std::array<Solution, GA::POPULATION_SIZE>, 2> populations;
    std::array<std::array<Solution, GA::POPULATION_SIZE>, 2> prev_populations;

public:
    Solution my_best, enemy_best;
    Solver();
    void solve(high_resolution_clock::time_point &start_time);
};

#endif //MINI_AI_CUP_3_SOLVER_H
