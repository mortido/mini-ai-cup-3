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

        previous[idx].reset_to(-1);
        evaluate(simulation, previous[idx], best_solutions, my_id, enemy_id);
        if (!best || (previous[idx].fitness > best->fitness)) {
            best = &previous[idx];
        }
        idx++;

        previous[idx].reset_to(1);
        evaluate(simulation, previous[idx], best_solutions, my_id, enemy_id);
        if (!best || (previous[idx].fitness > best->fitness)) {
            best = &previous[idx];
        }
        idx++;

        // fill with random chromosomes
        for (; idx < GA::POPULATION_SIZE; idx++) {
            previous[idx].randomize();
            evaluate(simulation, previous[idx], best_solutions, my_id, enemy_id);
            if (!best || (previous[idx].fitness > best->fitness)) {
                best = &previous[idx];
            }
        }
    }
#ifdef DEBUG
    int iteration =0;
#endif
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
#ifdef DEBUG
        iteration++;
#endif
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
    double mul(1.0), mul2(0.35);
    bool calc_for_bus = simulation.cars[0]->external_id == 2;

    for (int i = 0; i < GA::DEPTH; i++) {

//        if(i==GA::DEPTH/2){
//            cpSpaceSetIterations(simulation.space, 5);
//        }

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

        simulation.step();

        if (!simulation.cars[my_id]->alive) {
            while (i < GA::DEPTH) {
                fitness += -9000000.0 * mul;
                mul *= GA::THETA;
                i++;
            }
            break;
        }

        if (!simulation.cars[enemy_id]->alive) {
            fitness += 100000.0 * mul;
//            while (i < GA::DEPTH) {
//                fitness += 10000.0 * mul;
//                mul *= GA::THETA;
//                i++;
//            }
//            break;
        }

        if (calc_for_bus) {
            fitness += calcBusFitness(simulation, my_id, enemy_id, mul, mul2);
        } else {
            fitness += calcFitness(simulation, my_id, enemy_id, mul, mul2);
        }

        mul *= GA::THETA;
        mul2 *= GA::THETA_PLUS;
//        mul2 *= GA::THETA_PLUS;
//        mul2 *= GA::THETA_PLUS;
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

double Solver::calcFitness(Simulation &simulation, int my_id, int enemy_id, double mul, double mul2) {
    double fitness(0.0);

#ifdef OPTIMIZATION_RUN
    GameConstants *c = GameConstants::INSTANCE();

    double my_btn_dist_to_objects = simulation.get_closest_point_to_button(my_id);
    double enemy_btn_dist_to_objects = simulation.get_closest_point_to_button(enemy_id);
    double btn_y_diff = simulation.get_lowest_button_point(my_id) -
                        simulation.get_lowest_button_point(enemy_id);

    double enemy_angle = simulation.get_car_angle(enemy_id);
    double my_angle = simulation.get_car_angle(my_id);

    double end_game_coef = (double) std::min(GAME::TICK_TO_DEADLINE, simulation.sim_tick_index) / GAME::TICK_TO_DEADLINE;
    btn_y_diff *=end_game_coef;
    btn_y_diff *= c->a;

    double aim = simulation.get_my_distance_to_enemy_button(enemy_id,my_id)-simulation.get_my_distance_to_enemy_button(my_id, enemy_id);
    aim *= c->b * (1.0 - end_game_coef);

    my_btn_dist_to_objects *= c->c;
    my_btn_dist_to_objects = my_btn_dist_to_objects / (1.0 + abs(my_btn_dist_to_objects));
    my_btn_dist_to_objects *= c->d;

    enemy_btn_dist_to_objects *= c->e;

    enemy_angle = abs(enemy_angle) * c->f;
    my_angle = abs(my_angle);
    if (my_angle < PI / 2.0) {
        my_angle = 0;
    } else {
        my_angle *= c->g;
    }


    fitness += (my_btn_dist_to_objects - enemy_btn_dist_to_objects - my_angle + enemy_angle + aim) * mul +
               (btn_y_diff) * mul2;

    fitness += mul2 * simulation.get_position_score(my_id) * c->h;

#else
//    0  >>>>  25.2106 SOLUTION [
//            a = 12.395199375452764,
//            b = 6.130363505477251,
//            c = 0.017271487783766918,
//            d = 525.0464332949529,
//            e = 4.798823359684237,
//            f = 97.70336392301641,
//            g = 57.3477094451587,
//            h = 277.8335405164962]

//    a = 20.404509809204708,
//    b = 5.598132402016896,
//    c = 0.03035488833458294,
//    d = 294.3200379306266,
//    e = 10.647558480171222,
//    f = 42.87901827207437,
//    g = 117.84914839793149,
//    h = 268.572707611291]
    double my_btn_dist_to_objects = simulation.get_closest_point_to_button(my_id);
    double enemy_btn_dist_to_objects = simulation.get_closest_point_to_button(enemy_id);
    double btn_y_diff = simulation.get_lowest_button_point(my_id) -
                        simulation.get_lowest_button_point(enemy_id);

    double enemy_angle = simulation.get_car_angle(enemy_id);
    double my_angle = simulation.get_car_angle(my_id);

    double end_game_coef = (double) std::min(GAME::TICK_TO_DEADLINE, simulation.sim_tick_index) / GAME::TICK_TO_DEADLINE;
    btn_y_diff *=end_game_coef;
    btn_y_diff *= 20.4;

    double aim = simulation.get_my_distance_to_enemy_button( enemy_id,my_id)-simulation.get_my_distance_to_enemy_button(my_id, enemy_id);
    aim *= 5.6 * (1.0 - end_game_coef);
//    if (simulation.sim_tick_index >= GAME::TICK_TO_DEADLINE) {
//        aim += std::min(250.0, simulation.get_my_distance_to_enemy_button(enemy_id, my_id)) * mul * 2.0;
//    } else {
//        aim = -simulation.get_my_distance_to_enemy_button(my_id, enemy_id);
//        aim *= 2.5;
//    }


//    my_btn_dist_to_objects *= 1.0;
    my_btn_dist_to_objects *= 0.03;
    my_btn_dist_to_objects = my_btn_dist_to_objects / (1.0 + abs(my_btn_dist_to_objects));
    my_btn_dist_to_objects *= 294.3;

    enemy_btn_dist_to_objects *= 10.64;
//    enemy_btn_dist_to_objects *= 0.9;
    enemy_angle = abs(enemy_angle) * 42.8;
    my_angle = abs(my_angle);
    if (my_angle < PI / 2.0) {
        my_angle = 0;
    } else {
        my_angle *= 117.8;
    }


    fitness += (my_btn_dist_to_objects - enemy_btn_dist_to_objects - my_angle + enemy_angle + aim) * mul +
               (btn_y_diff) * mul2;

    fitness += mul2 * simulation.get_position_score(my_id) * 268.6;

#ifdef REWIND_VIEWER
    if (print_fitness) {
        simulation.rewind.message("%f %f %fd %fd %f %f\\n", my_btn_dist_to_objects, enemy_btn_dist_to_objects,
                                  enemy_angle, my_angle, aim, btn_y_diff);
        simulation.rewind.message("%f \\n",
                                  (my_btn_dist_to_objects - enemy_btn_dist_to_objects - my_angle + enemy_angle + aim) *
                                  mul + (btn_y_diff) * mul2);
        print_fitness = false;
    }
#endif

// **********************************
//        double min_dist = simulation.get_closest_point_to_button(my_id);
//        fitness += min_dist * mul;
//        min_dist = simulation.get_closest_point_to_button(enemy_id);
//        fitness += -min_dist * 0.9 * mul;
//        fitness += (simulation.get_lowest_button_point(my_id) - simulation.get_lowest_button_point(enemy_id)) * 1.9 * mul;
//        if (simulation.sim_tick_index >= GAME::TICK_TO_DEADLINE) {
//            fitness += std::min(250.0, simulation.get_my_distance_to_enemy_button(enemy_id, my_id)) * mul;
//        } else {
//            fitness += -simulation.get_my_distance_to_enemy_button(my_id, enemy_id) * mul;
//        }
#endif
    return fitness;
}

double Solver::calcBusFitness(Simulation &simulation, int my_id, int enemy_id, double mul, double mul2) {
    double fitness(0.0);

#ifdef OPTIMIZATION_RUN
#else

    double my_btn_dist_to_objects = simulation.get_closest_point_to_button(my_id);
    double enemy_btn_dist_to_objects = simulation.get_closest_point_to_button(enemy_id);
    double btn_y_diff = simulation.get_lowest_button_point(my_id) -
                        simulation.get_lowest_button_point(enemy_id);
    double position_on_map = simulation.get_position_score(my_id);


    btn_y_diff *= (double) std::min(GAME::TICK_TO_DEADLINE, simulation.sim_tick_index) / GAME::TICK_TO_DEADLINE;
    btn_y_diff *= 10;

    my_btn_dist_to_objects *= 0.02;
    my_btn_dist_to_objects = my_btn_dist_to_objects / (1.0 + abs(my_btn_dist_to_objects));
    my_btn_dist_to_objects *= 100;

    enemy_btn_dist_to_objects *= 3.6;

    position_on_map *= 1000;

    fitness += (my_btn_dist_to_objects - enemy_btn_dist_to_objects) * mul + (btn_y_diff + position_on_map) * mul2;

#ifdef REWIND_VIEWER
    if (print_fitness) {
        simulation.rewind.message("me_d: %f en_d: %f pos:%f diff:%f\\n", my_btn_dist_to_objects,
                                  enemy_btn_dist_to_objects,
                                  position_on_map, btn_y_diff);
        simulation.rewind.message("BUS: %f \\n",
                                  (my_btn_dist_to_objects - enemy_btn_dist_to_objects) * mul +
                                  (btn_y_diff + position_on_map) * mul2);
        print_fitness = false;
    }
#endif

// **********************************
//        double min_dist = simulation.get_closest_point_to_button(my_id);
//        fitness += min_dist * mul;
//        min_dist = simulation.get_closest_point_to_button(enemy_id);
//        fitness += -min_dist * 0.9 * mul;
//        fitness += (simulation.get_lowest_button_point(my_id) - simulation.get_lowest_button_point(enemy_id)) * 1.9 * mul;
//        if (simulation.sim_tick_index >= GAME::TICK_TO_DEADLINE) {
//            fitness += std::min(250.0, simulation.get_my_distance_to_enemy_button(enemy_id, my_id)) * mul;
//        } else {
//            fitness += -simulation.get_my_distance_to_enemy_button(my_id, enemy_id) * mul;
//        }
#endif
    return fitness;
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
