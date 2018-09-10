#include "Solver.h"

#define NOW high_resolution_clock::now()
#define ELAPSED_TIME duration_cast<duration<double>>(NOW - start_time).count()


void Solver::_solve(Simulation &simulation, high_resolution_clock::time_point &start_time,
                    double time_limit, int my_id, int enemy_id) {

#ifdef DEBUG
#define LIMIT iteration < GA::DEBUG_ITERATIONS_LIMIT
#else
#define LIMIT ELAPSED_TIME < time_limit
#endif

    Solution *best = nullptr;
    {
        // prepare populations
        population_t &previous = *population.previous;
        int idx = 0;

        // add best from prev move if exist
        if (freeze_my_move) {
            previous[0].copy_from(best_solutions[my_id]);
            evaluate(simulation, previous[0], best_solutions, my_id, enemy_id);
            best = &previous[0];
            idx++;
        }

        // fill with random chromosomes
        for (; idx < GA::POPULATION_SIZE; idx++) {
            previous[idx].randomize();
            evaluate(simulation, previous[idx], best_solutions, my_id, enemy_id);
            if (!best || (previous[idx].fitness > best->fitness)) {
                best = &previous[idx];
            }
        }
    }

    while (LIMIT) {
        population_t &current = *population.current;
        population_t &previous = *population.previous;

        // trick: copy best chromosome and mutate it
        current[0].copy_from(*best);
        current[0].mutate();

        evaluate(simulation, current[0], best_solutions, my_id, enemy_id);
        if (best->fitness < current[0].fitness) {
            best = &current[0];
        }

        // fill population with children
        for (int idx = 1; (idx < GA::POPULATION_SIZE) && (LIMIT); idx++) {
            auto parents = population.get_parent_indexes();
            current[idx].merge(previous[parents.first], previous[parents.second]);

            // Mutate with some chance.
            if (Randomizer::GetProbability() <= GA::MUTATION_PROBABILITY) {
                current[idx].mutate();
            }

            evaluate(simulation, current[idx], best_solutions, my_id, enemy_id);
            if (best->fitness < current[idx].fitness) {
                best = &current[idx];
            }
        }

        // swap prev and current populations, save best.
        population.swap();

//#ifdef LOCAL_RUN
        if (my_id == my_player_id) {
            my_generations++;
        } else {
            enemy_generations++;
        }
//#endif
    }

    best_solutions[my_id].copy_from(*best);
}

void Solver::solve(Simulation &simulation, high_resolution_clock::time_point &start_time,
                   double time_limit, int my_prev_move) {
//#ifdef LOCAL_RUN
    my_generations = 0;
    enemy_generations = 0;
    my_simulations = 0;
    enemy_simulations = 0;
//#endif


    double my_TL = time_limit;
    double enemy_TL = GA::ENEMY_TIME_COEFF * time_limit;
    freeze_my_move = simulation.saved_tick > 0;
    frozen_move = my_prev_move;

    // shift previous best moves
    if (simulation.saved_tick > 0) {
        for (int player = 0; player < PLAYERS_COUNT; player++) {
            best_solutions[player].shift();
        }
    }

    _solve(simulation, start_time, enemy_TL, enemy_player_id, my_player_id);
    _solve(simulation, start_time, my_TL, my_player_id, enemy_player_id);
}

void Solver::evaluate(Simulation &simulation,
                      Solution &test_solution,
                      std::array<Solution, PLAYERS_COUNT> &best_sols,
                      int my_id, int enemy_id) {

    simulation.restore();
    double fitness(0.0);
    double mul(1.0), mul2(1.0);

    for (int i = 0; i < GA::DEPTH; i++) {
        // Apply moves.
        int m = 0;
        for (int p = 0; p < PLAYERS_COUNT; p++) {
            if (i == 0 && p == my_player_id && freeze_my_move) {
                m = frozen_move;
            } else {
                m = p == my_id ? test_solution.moves[i] : best_sols[p].moves[i];
            }

            simulation.cars[p]->move(m);
        }

        // TODO: adaptive step DT (late steps less precise?)
        simulation.step();

#ifdef OPTIMIZATION_RUN
        GameConstants *c = GameConstants::INSTANCE();

        if (!simulation.cars[my_id]->alive) {
            while (i < GA::DEPTH) {
                fitness += -9000.0 * mul;
                mul *= GA::THETA;
                i++;
            }
            break;
        } else {
            double min_dist = simulation.get_closest_point_to_button(my_id);
            if (c->use_sigmoid) {
                min_dist = c->my_danger_sig_coeff * (min_dist - c->my_danger_shift);
                min_dist = min_dist / (1.0 + abs(min_dist));
            }

            if (c->my_danger_theta == 1) {
                min_dist *= mul;
            } else if (c->my_danger_theta == 2) {
                min_dist *= mul2;
            }

            fitness += min_dist * c->my_danger_coeff;
        }

        if (!simulation.cars[enemy_id]->alive) {
            while (i < GA::DEPTH) {
                fitness += 9000.0 * mul;
                mul *= GA::THETA;
                i++;
            }
            break;
        } else {
            double min_dist = simulation.get_closest_point_to_button(enemy_id);
            if (c->use_sigmoid) {
                min_dist = c->enemy_danger_sig_coeff * (min_dist - c->enemy_danger_shift);
                min_dist = min_dist / (1.0 + abs(min_dist));
            }

            if (c->enemy_danger_theta == 1) {
                min_dist *= mul;
            } else if (c->enemy_danger_theta == 2) {
                min_dist *= mul2;
            }

            fitness -= min_dist * c->enemy_danger_coeff;
        }

        double aim = 0;
        if (c->aim_type && (simulation.sim_tick_index >= GAME::TICK_TO_DEADLINE || c->aim_type==3)) {
            aim = simulation.get_button_lowest_position(my_id);
            if (c->aim_type == 2) {
                aim -= simulation.get_button_lowest_position(enemy_id);
            }

            if (c->use_sigmoid) {
                aim = c->aim_safety_sig_coeff * (aim - c->aim_safety_shift);
                aim = aim / (1.0 + abs(aim));
            }
            aim *= c->aim_safety_coeff;

            if(c->aim_type==3){
                double aim2 = simulation.get_my_distance_to_enemy_button(my_id, enemy_id);
                if (c->use_sigmoid) {
                    aim2 = c->aim_attack_sig_coeff * (aim2 - c->aim_attack_shift);
                    aim2 = aim2 / (1.0 + abs(aim2));
                }
                aim2 *= - c->aim_attack_coeff;
                aim += aim2;
            }
        } else {
            aim = simulation.get_my_distance_to_enemy_button(my_id, enemy_id);
            if (c->use_sigmoid) {
                aim = c->aim_attack_sig_coeff * (aim - c->aim_attack_shift);
                aim = aim / (1.0 + abs(aim));
            }
            aim *= - c->aim_attack_coeff;
        }

        if (c->aim_theta == 1) {
            aim *= mul;
        } else if (c->aim_theta == 2) {
            aim *= mul2;
        }

        fitness += aim;

#else
        if (!simulation.cars[my_id]->alive) {
            while (i < GA::DEPTH) {
                fitness += -9000.0 * mul;
                mul *= GA::THETA;
                i++;
            }
            break;
        } else {
            double min_dist = simulation.get_closest_point_to_button(my_id);
            fitness += min_dist;
            // TODO: min_dist = b * (min_dist - a)
//            fitness += min_dist / (1.0 + abs(min_dist)) * mul;
        }

        if (!simulation.cars[enemy_id]->alive) {
            while (i < GA::DEPTH) {
                fitness += 9000.0 * mul;
                mul *= GA::THETA;
                i++;
            }
            break;
        } else {
            double min_dist = simulation.get_closest_point_to_button(enemy_id);
            fitness += -min_dist;
//            // TODO: min_dist = (min_dist - a) / b
//            fitness += -min_dist *1.5/ (1.0 + abs(min_dist)) * mul;
        }

//        if (simulation.sim_tick_index >= GAME::TICK_TO_DEADLINE) {
//            cpBody *c = simulation.cars[my_id]->car_body;
//            fitness += cpBodyLocalToWorld(c, cpBodyGetCenterOfGravity(c)).y;
//        } else {
            fitness += -simulation.get_my_distance_to_enemy_button(my_id, enemy_id) * mul;
//        }
#endif

        mul *= GA::THETA;
        mul2 *= GA::THETA_PLUS;
    }

    test_solution.fitness = fitness;

//#ifdef LOCAL_RUN
    if (my_id == my_player_id) {
        my_simulations++;
    } else {
        enemy_simulations++;
    }
//#endif
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
