#ifndef MINI_AI_CUP_3_SOLUTION_H
#define MINI_AI_CUP_3_SOLUTION_H

#include "constants.h"

class solution {
public:
    double fitness;
    int moves[GA::DEPTH];

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

    void mutate();

    void randomize();
};


#endif //MINI_AI_CUP_3_SOLUTION_H
