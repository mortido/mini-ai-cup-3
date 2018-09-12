#include "Solution.h"

// DIFFERENT VARIATIONS

void Solution::merge_shuffle(Solution &solution1, Solution &solution2) {
    for (int i = 0; i < GA::DEPTH; i++) {
        if (Randomizer::FlipCoin()) {
            moves[i] = solution1.moves[i];
        } else {
            moves[i] = solution2.moves[i];
        }
    }
}

void Solution::merge_crossover(Solution &solution1, Solution &solution2) {
    int crossover_point = Randomizer::GetCrossoverPoint();
    for (int i = 0; i < crossover_point; i++) {
        moves[i] = solution1.moves[i];
    }
    for (int i = crossover_point; i < GA::DEPTH; i++) {
        moves[i] = solution2.moves[i];
    }
}

void Solution::mutate_single_gen() {
    moves[Randomizer::GetMutatePoint()] = Randomizer::GetRandomMove();
}

void Solution::mutate_slice() {
    int m = Randomizer::GetRandomMove();
    for (int i = Randomizer::GetMutatePoint(); (i < GA::DEPTH); i++) {
        moves[i] = m;

        if (Randomizer::GetProbability() < GA::STOP_SLICE_MUTATE_PROBABILITY) {
            break;
        }
    }
}

void Solution::mutate_slice_all_random() {
    for (int i = Randomizer::GetMutatePoint(); (i < GA::DEPTH); i++) {
        moves[i] = Randomizer::GetRandomMove();

        if (Randomizer::GetProbability() < GA::STOP_SLICE_MUTATE_PROBABILITY) {
            break;
        }
    }
}

void Solution::randomize_interval() {
    int m = Randomizer::GetRandomMove();
    for (int i = 0; i < GA::DEPTH; i++) {
        moves[i] = m;

        if (Randomizer::GetProbability() < GA::CHANGE_MOVE_ON_RANDOMIZATION_PROBABILITY) {
            m = Randomizer::GetRandomMove();
        }
    }
}

void Solution::randomize_shuffle() {
    for (int i = 0; i < GA::DEPTH; i++) {
        moves[i] = Randomizer::GetRandomMove();
    }
}

// --------------------------------------------------------------------

void Solution::shift() {
    for (int i = 0; i < GA::DEPTH - 1; i++) {
        moves[i] = moves[i + 1];
    }
    moves[GA::DEPTH - 1] = Randomizer::GetRandomMove();
}

json Solution::to_json(int move_index) {
    json command;
    switch (moves[move_index]) {
        case 0:
            command["command"] = "stop";
            break;
        case -1:
            command["command"] = "left";
            break;
        case 1:
            command["command"] = "right";
            break;
        default:
            command["command"] = "stop";
    }

    return command;
}

void Solution::copy_from(Solution &solution) {
    for (int i = 0; i < GA::DEPTH; i++) {
        moves[i] = solution.moves[i];
    }
    fitness = solution.fitness;
}

void Solution::randomize() {

//    randomize_interval();
    randomize_shuffle();

}

void Solution::mutate() {
        mutate_slice();
}

void Solution::merge(Solution &solution1, Solution &solution2) {
    if (Randomizer::FlipCoin()) {
        merge_crossover(solution1, solution2);
    } else {
        merge_crossover(solution2, solution1);
    }
}
