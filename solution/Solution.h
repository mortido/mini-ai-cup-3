#ifndef MINI_AI_CUP_3_SOLUTION_H
#define MINI_AI_CUP_3_SOLUTION_H

#include <array>
#include <memory>

#include "Constants.h"
#include "random/Randomizer.h"

#include "../../nlohmann/json.hpp"

using json = nlohmann::json;


class Solution {
public:
    double fitness;

    std::array<double,10> fitness_components{};

    std::array<int, GA::DEPTH> moves;

    void randomize();
    void reset_to(int move);
    void reset_to_butt_move(int player_id);
    void merge(Solution &solution1, Solution &solution2);
    void mutate();
    void shift();
    void copy_from(Solution &solution);

    json to_json(int move_index);

private:
    void randomize_interval();
    void randomize_shuffle();
    void merge_shuffle(Solution &solution1, Solution &solution2);
    void merge_crossover(Solution &solution1, Solution &solution2);
    void mutate_single_gen();
    void mutate_slice();
    void mutate_slice_all_random();
};


#endif //MINI_AI_CUP_3_SOLUTION_H
