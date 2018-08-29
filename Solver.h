#ifndef MINI_AI_CUP_3_SOLVER_H
#define MINI_AI_CUP_3_SOLVER_H

#include <chrono>
#include <array>

#include "Solution.h"
#include "GameState.h"


using namespace std::chrono;

#define PLAYERS_COUNT 2

typedef std::array<Solution, GA::POPULATION_SIZE> population_t;

class Solver {
private:
    // pool that contain all previous and current populations for all players
    std::array<population_t, 2 * PLAYERS_COUNT> populations_pool;

    // pointers to current and previous population to make possible swap them without making copies.
    std::array<population_t*, PLAYERS_COUNT> curr_populations;
    std::array<population_t*, PLAYERS_COUNT> prev_populations;

    void evaluate(Solution &my_solution, std::array<Solution, PLAYERS_COUNT> &best_solutions, int my_id);
public:
    std::array<Solution, PLAYERS_COUNT> best_solutions;
    Solver();
    void solve(GameState &game_state, high_resolution_clock::time_point &start_time);
};

#endif //MINI_AI_CUP_3_SOLVER_H
