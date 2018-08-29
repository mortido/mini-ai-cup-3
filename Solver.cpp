#include "Solver.h"

#define NOW high_resolution_clock::now()
#define ELAPSED_TIME duration_cast<duration<double>>(NOW - start_time).count()

Solver::Solver() {}

void Solver::solve(GameState &game_state, high_resolution_clock::time_point &start_time) {
    int iteration = 1;

#define LIMIT ELAPSED_TIME < GA::TIME_LIMIT

    // shift previous best moves
    if (game_state.tick_index > 0) {
        for (int player = 0; player < PLAYERS_COUNT; player++) {
            best_solutions[player].shift();
        }
    }

    // prepare populations (my player last)
    for (auto player:{game_state.enemy_player_id, game_state.my_player_id}) {

        population_t &current = *population_states[player].current;
        population_t &previous = *population_states[player].previous;
        Solution *best = nullptr;

        int idx = 0;
        // add best from prev move if exist
        if (game_state.tick_index > 0) {
            best_solutions[player].shift();
            previous[0].copy_from(best_solutions[player]);

            evaluate(previous[0], best_solutions, player);
            best = &previous[0];

            idx++;
        }

        // fill with random chromosomes
        for (; idx < GA::POPULATION_SIZE; idx++) {
            previous[idx].randomize();

            evaluate(previous[idx], best_solutions, player);
            if (!best || previous[idx].fitness > best->fitness) {
                best = &previous[idx];
            }
        }

        best_solutions[player].copy_from(*best);
    }

    while (LIMIT) {
        // TODO: try not optimize enemy on last iteration (maybe set max_iter/time for them)...
        int solve_for_id = (iteration % GA::SOLVE_ENEMY_EVERY_N_TURNS)
                           ? game_state.my_player_id : game_state.enemy_player_id;

        population_t &current = *population_states[solve_for_id].current;
        population_t &previous = *population_states[solve_for_id].previous;

        Solution *best = &best_solutions[solve_for_id];

        // trick: copy best chromosome and mutate it.
        current[0].copy_from(*best);
        current[0].mutate();
        evaluate(current[0], best_solutions, solve_for_id);
        if (best->fitness < current[0].fitness) {
            best = &current[0];
        }

        // fill population with children
        for (int idx = 1; (idx < GA::POPULATION_SIZE) && (LIMIT); idx++) {

            auto parents = population_states[solve_for_id].get_parent_indexes();
            current[idx].merge(previous[parents.first], previous[parents.second]);
// TODO:
//                if (rnd.nextDouble() <= mutationProb) {
            current[idx].mutate();
//                }
            evaluate(current[idx], best_solutions, solve_for_id);
            if (best->fitness < current[idx].fitness) {
                best = &current[idx];
            }
        }

        // swap prev and current populations, save best.
        population_states[solve_for_id].swap();
        best_solutions[solve_for_id].copy_from(*best);
        iteration++;
    }
}

void Solver::evaluate(Solution &test_solution, std::array<Solution, PLAYERS_COUNT> &best_solutions, int my_id) {
    test_solution.fitness = 0;
}

Solver::PopulationState::PopulationState() {
    current = std::unique_ptr<population_t>(new population_t());
    previous = std::unique_ptr<population_t>(new population_t());
}

void Solver::PopulationState::swap() {
    std::swap(current, previous);
}