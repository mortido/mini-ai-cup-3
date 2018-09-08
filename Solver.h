#ifndef MINI_AI_CUP_3_SOLVER_H
#define MINI_AI_CUP_3_SOLVER_H

#include <chrono>
#include <array>
#include "random/Randomizer.h"

#include "Solution.h"
#include "simulation/Simulation.h"


using namespace std::chrono;

#define PLAYERS_COUNT 2

using population_t = std::array<Solution, GA::POPULATION_SIZE>;
using population_ptr = std::unique_ptr<population_t>;


class Solver {
private:
    // **********************
    class PopulationState {
    public:
        population_ptr current, previous;

        PopulationState();

        void swap();

        std::pair<int, int> get_parent_indexes();
    };
    // **********************

    std::array<PopulationState, PLAYERS_COUNT> population_states;
    std::array<std::pair<int,int>, PLAYERS_COUNT> prepare_order;
    int my_frozen_move;
    bool froze_move;

    void evaluate(Simulation &simulation,
                  Solution &test_solution,
                  std::array<Solution, PLAYERS_COUNT> &best_solutions,
                  int my_player_id, int enemy_player_id);

public:
    std::array<Solution, PLAYERS_COUNT> best_solutions;

    double time_limit;

    void solve(Simulation &simulation, high_resolution_clock::time_point &start_time,
               int my_player_id, int enemy_player_id);
    void new_round(double time_bank, int my_lives, int enemy_lives);
        void new_tick(int tick_index, int my_prev_move);

#ifdef LOCAL_RUN
    int simulations;
#endif
};

#endif //MINI_AI_CUP_3_SOLVER_H
