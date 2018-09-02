#include "Solver.h"

#define NOW high_resolution_clock::now()
#define ELAPSED_TIME duration_cast<duration<double>>(NOW - start_time).count()

#ifdef DEBUG
#define LIMIT iteration < GA::DEBUG_ITERATIONS_LIMIT
#else
#define LIMIT ELAPSED_TIME < time_limit
#endif

void Solver::solve(Simulation &simulation, high_resolution_clock::time_point &start_time) {
    int iteration = 0;
    simulations = 0;

    // shift previous best moves
    if (simulation.tick_index > 0) {
        for (int player = 0; player < PLAYERS_COUNT; player++) {
            best_solutions[player].shift();
        }
    }

    // prepare populations (my player last)
    for (auto player: prepare_order) {
//        population_t &current = *population_states[player].current;
        population_t &previous = *population_states[player].previous;
        Solution *best = nullptr;

        int idx = 0;
        // add best from prev move if exist
        if (simulation.tick_index > 0) {
            best_solutions[player].shift();
            previous[0].copy_from(best_solutions[player]);

            evaluate(simulation, previous[0], best_solutions, player);
            best = &previous[0];

            idx++;
        }

        // fill with random chromosomes
        for (; idx < GA::POPULATION_SIZE; idx++) {
            previous[idx].randomize();

            evaluate(simulation, previous[idx], best_solutions, player);
            if (!best || (previous[idx].fitness > best->fitness)) {
                best = &previous[idx];
            }
        }

        best_solutions[player].copy_from(*best);
    }

    int prev_solve_id = simulation.my_player_id;
    while (LIMIT) {
        // TODO: try not optimize enemy on last iteration (maybe set max_iter/time for them)...
        int solve_for_id = (iteration % GA::SOLVE_ENEMY_EVERY_N_TURNS)
                           ? simulation.my_player_id : simulation.enemy_player_id;

        population_t &current = *population_states[solve_for_id].current;
        population_t &previous = *population_states[solve_for_id].previous;

        Solution *best = &best_solutions[solve_for_id];

        if (solve_for_id != prev_solve_id) {
            // re-evaluate best solution as enemies was updated too
            // TODO: may be track that enemies was actually updated...
            evaluate(simulation, *best, best_solutions, solve_for_id);
            prev_solve_id = solve_for_id;
        }

        // trick: copy best chromosome and mutate it
        current[0].copy_from(*best);
        current[0].mutate();
        evaluate(simulation, current[0], best_solutions, solve_for_id);
        if (best->fitness < current[0].fitness) {
            best = &current[0];
        }

        // fill population with children
        for (int idx = 1; (idx < GA::POPULATION_SIZE) && (LIMIT); idx++) {

            auto parents = population_states[solve_for_id].get_parent_indexes();
            current[idx].merge(previous[parents.first], previous[parents.second]);

            // Mutate with some chance.
            if (Randomizer::GetProbability() <= GA::MUTATION_PROBABILITY) {
                current[idx].mutate();
            }

            evaluate(simulation, current[idx], best_solutions, solve_for_id);
            if (best->fitness < current[idx].fitness) {
                best = &current[idx];
            }
        }

        // swap prev and current populations, save best.
        population_states[solve_for_id].swap();
        best_solutions[solve_for_id].copy_from(*best);
        iteration++;
    }
//    std::cout << iteration << std::endl;
}

void Solver::evaluate(Simulation &simulation,
                      Solution &test_solution,
                      std::array<Solution, PLAYERS_COUNT> &best_solutions,
                      int my_id) {

    // TODO: reset simulation.

    // TODO: simulate
    for (int i = 0; i < GA::DEPTH; i++) {
        // TODO: adaptive step DT (late steps less precise)
//        simulation.simulate_tick();
    }

    // TODO: evaluate position

    simulations++;
    test_solution.fitness = Randomizer::GetProbability();
}

void Solver::init(Simulation &simulation) {
    prepare_order[0] = simulation.enemy_player_id;
    prepare_order[1] = simulation.my_player_id;
}

// ********************* POPULATION STATES *********************

Solver::PopulationState::PopulationState() {
    current = std::unique_ptr<population_t>(new population_t());
    previous = std::unique_ptr<population_t>(new population_t());
}

void Solver::PopulationState::swap() {
    std::swap(current, previous);
}

std::pair<int, int> Solver::PopulationState::get_parent_indexes() {
    int a = Randomizer::GetRandomParent();
    int b;

    do {
        b = Randomizer::GetRandomParent();
    } while (b == a);

    int mom = (*previous)[a].fitness > (*previous)[b].fitness ? a : b;

    do {
        a = Randomizer::GetRandomParent();
    } while (a == mom);

    do {
        b = Randomizer::GetRandomParent();
    } while (b == a || b == mom);

    int dad = (*previous)[a].fitness > (*previous)[b].fitness ? a : b;

    return std::make_pair(mom, dad);
}
