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

    PopulationState population;
    int frozen_move;
    bool freeze_my_move;

    void _solve(Simulation &simulation,
                high_resolution_clock::time_point &start_time,
                double time_limit,
                int my_id, int enemy_id);

    void evaluate(Simulation &simulation,
                  Solution &test_solution,
                  std::array<Solution, PLAYERS_COUNT> &best_solutions,
                  int my_player_id, int enemy_player_id);

public:
    std::array<Solution, PLAYERS_COUNT> best_solutions;
    int my_player_id, enemy_player_id;


    void solve(Simulation &simulation, high_resolution_clock::time_point &start_time,
               double time_limit, int my_prev_move);

    void calcBuggyFitness(Simulation &simulation, Solution &solution, int my_id, int enemy_id, double mul, double mul2);
    void calcSquareFitness(Simulation &simulation, Solution &solution, int my_id, int enemy_id, double mul, double mul2);
    void calcBusFitness(Simulation &simulation, Solution &solution, int my_id, int enemy_id, double mul, double mul2);

//#ifdef LOCAL_RUN
    int my_simulations, enemy_simulations;
    int my_generations, enemy_generations;
//#endif
};

#endif //MINI_AI_CUP_3_SOLVER_H
