#ifndef MINI_AI_CUP_3_SOLUTION_H
#define MINI_AI_CUP_3_SOLUTION_H

#include "constants.h"

#include "../nlohmann/json.hpp"

using json = nlohmann::json;


class Solution {
public:
    double fitness;
    int moves[GA::DEPTH];

    Solution* merge_shuffle(Solution* solution);

    Solution* merge_crossover(Solution* solution);

    void mutate_single_gen();

    void mutate_slice();

    void mutate_slice_all_random();

    void randomize();

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
