#ifndef MINI_AI_CUP_3_SOLUTION_H
#define MINI_AI_CUP_3_SOLUTION_H

#include <array>

#include "c_onstants.h"

#include "../nlohmann/json.hpp"

using json = nlohmann::json;


class Solution {
public:
    double fitness;
    std::array<int, GA::DEPTH> moves;

    Solution* merge_shuffle(Solution* solution);

    Solution* merge_crossover(Solution* solution);

    void mutate_single_gen();

    void mutate_slice();

    void mutate_slice_all_random();

    void randomize();

    void randomize_interval();

    void shit();

    json to_json();

//    solution* merge(solution* solution) {
//        solution* child = new solution();
//
//        for (int i = 0; i < DEPTH; ++i) {
//            if (fastRandInt(2)) {
//                child->moves1[i] = solution->moves1[i];
//                child->moves2[i] = solution->moves2[i];
//            } else {
//                child->moves1[i] = moves1[i];
//                child->moves2[i] = moves2[i];
//            }
//        }
//
//
//        return child;
//    }
//
//    void copy(solution* solution) {
//        for (int i = 0; i < DEPTH; ++i) {
//            moves[i] = solution->moves[i];
//        }
//
//        this->points = solution->points;
//    }
};


#endif //MINI_AI_CUP_3_SOLUTION_H
