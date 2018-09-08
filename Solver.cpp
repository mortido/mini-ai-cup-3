#include "Solver.h"

#define NOW high_resolution_clock::now()
#define ELAPSED_TIME duration_cast<duration<double>>(NOW - start_time).count()

#ifdef DEBUG
#define LIMIT iteration < GA::DEBUG_ITERATIONS_LIMIT
#else
#define LIMIT ELAPSED_TIME < time_limit
#endif

void Solver::solve(Simulation &simulation, high_resolution_clock::time_point &start_time,
                   int my_player_id, int enemy_player_id) {
    int iteration = 0;


    prepare_order[0] = std::make_pair(enemy_player_id, my_player_id);
    prepare_order[1] = std::make_pair(my_player_id, enemy_player_id);

    // shift previous best moves
    if (froze_move) {
        for (int player = 0; player < PLAYERS_COUNT; player++) {
            best_solutions[player].shift();
        }
    }

    // prepare populations (my player last)
    for (auto players: prepare_order) {
//        population_t &current = *population_states[player].current;
        population_t &previous = *population_states[players.first].previous;
        Solution *best = nullptr;

        int idx = 0;
        // add best from prev move if exist
        if (froze_move) {
            best_solutions[players.first].shift();
            previous[0].copy_from(best_solutions[players.first]);

            evaluate(simulation, previous[0], best_solutions, players.first,players.second);
            best = &previous[0];

            idx++;
        }

        // fill with random chromosomes
        for (; idx < GA::POPULATION_SIZE; idx++) {
            previous[idx].randomize();

            evaluate(simulation, previous[idx], best_solutions, players.first,players.second);
            if (!best || (previous[idx].fitness > best->fitness)) {
                best = &previous[idx];
            }
        }

        best_solutions[players.first].copy_from(*best);
    }

    int prev_solve_id = my_player_id;
    while (LIMIT) {
        // TODO: try not optimize enemy on last iteration (maybe set max_iter/time for them)...
//        int solve_for_id = (iteration % GA::SOLVE_ENEMY_EVERY_N_TURNS)
//                           ? my_player_id : enemy_player_id;
        auto players =  (iteration % GA::SOLVE_ENEMY_EVERY_N_TURNS) ? prepare_order[1] : prepare_order[0];
        int solve_for_id = players.first;

        population_t &current = *population_states[solve_for_id].current;
        population_t &previous = *population_states[solve_for_id].previous;

        Solution *best = &best_solutions[solve_for_id];

        if (solve_for_id != prev_solve_id) {
            // re-evaluate best solution as enemies was updated too
            // TODO: may be track that enemies was actually updated...
            evaluate(simulation, *best, best_solutions, players.first,players.second);
            prev_solve_id = solve_for_id;
        }

        // trick: copy best chromosome and mutate it
        current[0].copy_from(*best);
        current[0].mutate();
        evaluate(simulation, current[0], best_solutions,players.first,players.second);
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

            evaluate(simulation, current[idx], best_solutions, players.first,players.second);
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

void Solver::evaluate(Simulation &simulation,
                      Solution &test_solution,
                      std::array<Solution, PLAYERS_COUNT> &best_solutions,
                      int my_id, int enemy_id) {

    simulation.restore();
    double mul(1.0);
    double fitness(0.0);

//    // TODO: simulate
//    for (int i = 0; i < GA::DEPTH; i++) {
//
//        // Apply moves.
//        for (int p = 0; p < PLAYERS_COUNT; p++) {
//            simulation.cars[p]->move(p == my_id ? test_solution.moves[i] : best_solutions[p].moves[i]);
//        }
//
//        // TODO: adaptive step DT (late steps less precise?)
//        // if all alive:
////        simulation.simulate_tick();
//
//        // in case of somebody death.
//        if (!simulation.cars[0]->alive || !simulation.cars[1]->alive) {
//            double points = simulation.cars[my_id]->alive ? 100500 : -100500;
//            while (i < GA::DEPTH) {
//                test_solution.fitness += points * mul;
//                mul *= GA::THETA;
//            }
//            break;
//        }
//
//        // TODO: evaluate position
//        double min_dist = simulation.get_closest_point_to_button(my_id);
//
//        min_dist = simulation.get_closest_point_to_button(enemy_id);
//
//        fitness += Randomizer::GetProbability() * mul;
//        mul *= GA::THETA;
//    }

    test_solution.fitness = fitness;


#ifdef LOCAL_RUN
    simulations++;
#endif
}

void Solver::new_tick(int tick_index, int my_prev_move) {
    froze_move = tick_index > 0;
    my_frozen_move = my_prev_move;
}

void Solver::new_round(double time_bank, int my_lives, int enemy_lives) {
    time_limit = time_bank / ((my_lives + enemy_lives - 1) * GAME::MAX_ROUND_TICKS);
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
