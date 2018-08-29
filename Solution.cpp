#include "Solution.h"

// DIFFERENT VARIATIONS

void Solution::merge_shuffle(Solution &solution1, Solution& solution2) {

}

void Solution::merge_crossover(Solution &solution1, Solution& solution2) {

}

void Solution::mutate_single_gen() {

}

void Solution::mutate_slice() {

}

void Solution::mutate_slice_all_random() {

}

void Solution::randomize_interval() {

}

void Solution::randomize_shuffle() {

}

// --------------------------------------------------------------------

void Solution::shift() {

}

json Solution::to_json() {
    return json();
}

void Solution::copy_from(Solution &solution) {

}

void Solution::randomize() {
    randomize_interval();
}

void Solution::mutate() {
    mutate_slice();
}

void Solution::merge(Solution &solution1, Solution &solution2) {
    merge_crossover(solution1, solution2);
}
