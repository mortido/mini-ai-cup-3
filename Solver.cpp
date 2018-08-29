#include "Solver.h"

#define NOW high_resolution_clock::now()
#define ELAPSED_TIME duration_cast<duration<double>>(NOW - start_time).count()

Solver::Solver() {
    // Prepare pointers for populations.
    for (int i = 0; i < 2; i++) {
        curr_populations[i] = &populations_pool[2 * i];
        prev_populations[i] = &populations_pool[2 * i + 1];
    }
}

void Solver::solve(GameState &game_state, high_resolution_clock::time_point &start_time) {
    int iteration = 0;

#define LIMIT ELAPSED_TIME < GA::TIME_LIMIT

    { // Prepare populations
        int idx = 0;
        if (game_state.tick_index > 0) {
            // Shift previous best solutions.
            for (auto prev_best:best_solutions) {
                prev_best.shift();
            }

            for (int i = 0; i < PLAYERS_COUNT; i++) {
                population_t &p = (*prev_populations[i]);
                p[0].copy(best_solutions[i])
            }
            idx++;
        }
        for(;idx<GA::POPULATION_SIZE; idx++){

        }
    }

    while (LIMIT) {
        int solve_for_id = (iteration % GA::SOLVE_ENEMY_EVERY_N_TURNS)
                           ? game_state.my_player_id : game_state.enemy_player_id;

        // One more trick from learned from Margus
        // TODO: Copy best chromosome and mutate it.

        for (int idx = 1; (idx < GA::POPULATION_SIZE) && (LIMIT); idx++) {
/*
                int mom = getMom();
                int dad = getDad(mom);

                c = prevPopulation[mom].merge2(prevPopulation[dad]);

                if (rnd.nextDouble() <= mutationProb) {
                    c.mutate2();
                }
                world.simulate(c, playerId, false);

                if (bestChromosome.fitness < c.fitness) {
                    bestChromosome = c;
                    bestOnGeneration = generations;
                }
                population[idx] = c;
 */
        }


        swap(curr_populations[solve_for_id], prev_populations[solve_for_id]);
        iteration++;
    }
}

void Solver::evaluate(Solution &my_solution, std::array<Solution, PLAYERS_COUNT> &best_solutions, int my_id) {
    my_solution.fitness = 0;
}
