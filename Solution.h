#ifndef MINI_AI_CUP_3_SOLUTION_H
#define MINI_AI_CUP_3_SOLUTION_H

#include <array>

#include "Constants.h"

#include "../nlohmann/json.hpp"

using json = nlohmann::json;


class Solution {
public:
    double fitness;

    std::array<int, GA::DEPTH> moves;

    void merge_shuffle(Solution& solution1, Solution& solution2);

    void merge_crossover(Solution& solution1, Solution& solution2);

    void mutate_single_gen();

    void mutate_slice();

    void mutate_slice_all_random();

    void randomize();

    void randomize_interval();

    void shift();

    json to_json();
};


#endif //MINI_AI_CUP_3_SOLUTION_H
