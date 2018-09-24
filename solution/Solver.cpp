#include <chipmunk/chipmunk_structs.h>
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
            previous[0].fitness *= 1.005;
#ifdef REWIND_VIEWER
            if (my_id == my_player_id) {
                simulation.rewind.message("PREV_BEST: %.2f\\n", previous[0].fitness);
            }
#endif
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

//        if(simulation.cars[0]->external_id==2){
//            previous[idx].reset_to(0);
//            evaluate(simulation, previous[idx], best_solutions, my_id, enemy_id);
//            if (!best || (previous[idx].fitness > best->fitness)) {
//                best = &previous[idx];
//            }
//            idx++;
//        }

        // fill with random chromosomes
        for (; idx < GA::POPULATION_SIZE; idx++) {
            previous[idx].randomize();
            evaluate(simulation, previous[idx], best_solutions, my_id, enemy_id);
            if (!best || (previous[idx].fitness > best->fitness)) {
                best = &previous[idx];
            }
        }
    }
    best_solutions[my_id].copy_from(*best);
#ifdef DEBUG
    int iteration =0;
#endif
    while (LIMIT) {
        best = &best_solutions[my_id];
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

        best_solutions[my_id].copy_from(*best);
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
    double mul(1.0), mul2(0.35);//007);
    int car_id = simulation.cars[0]->external_id;
    for (int j = 0; j < 10; j++) {
        test_solution.fitness_components[j] = 0.0;
    }

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

        simulation.step();

        if (!simulation.cars[enemy_id]->alive) {
            test_solution.fitness_components[1] += 1000000.0 * mul;
            simulation.cars[enemy_id]->alive = true;
//            while (i < GA::DEPTH) {
//                fitness += 10000.0 * mul;
//                mul *= GA::THETA;
//                i++;
//            }
//            break;
        }

        if (!simulation.cars[my_id]->alive) {
            while (i < GA::DEPTH) {
                test_solution.fitness_components[0] += -9000000.0 * mul;
                mul *= GA::THETA;
                i++;
            }
            break;
        }

        switch (car_id) {
            case 1:
                calcBuggyFitness(simulation, test_solution, my_id, enemy_id, mul, mul2);
                break;
            case 2:
                calcBusFitness(simulation, test_solution, my_id, enemy_id, mul, mul2);
                break;

            case 3:
                calcSquareFitness(simulation, test_solution, my_id, enemy_id, mul, mul2);
                break;
        }

        mul *= GA::THETA;
        mul2 *= GA::THETA_PLUS;
    }
    test_solution.fitness = 0.0;
    for (int j = 0; j < 10; j++) {
        test_solution.fitness += test_solution.fitness_components[j];
    }

//#ifdef LOCAL_RUN
    if (my_id == my_player_id) {
        my_simulations++;
    } else {
        enemy_simulations++;
    }
//#endif
}


void
Solver::calcBuggyFitness(Simulation &simulation, Solution &solution, int my_id, int enemy_id, double mul, double mul2) {

    double my_danger = simulation.get_closest_point_to_button(my_id);
    double enemy_danger = simulation.get_closest_point_to_button(enemy_id);

    constexpr double dng_threshold = 52.0;
    if (my_danger > dng_threshold) {
        my_danger = dng_threshold;
    }

    if (my_danger < 3.0) {
        solution.fitness_components[9] -= 100000.0;
    }

    my_danger *= 0.15;
    my_danger = my_danger / (1.0 + abs(my_danger));
    my_danger *= 2500;

    if (enemy_danger > dng_threshold) {
        enemy_danger = dng_threshold;
    }
    enemy_danger *= -30.0; // 50 = 300, 75 = 450

    double my_danger_2 = simulation.get_closest_point_to_button2(my_id);
    double enemy_danger_2 = simulation.get_closest_point_to_button2(enemy_id);
    if (my_danger_2 < 3.0) {
        solution.fitness_components[9] -= 100000.0;
    }
    my_danger_2 *= 25;
    enemy_danger_2 *= -20.0; // 50 = 300, 75 = 450
    my_danger += my_danger_2;
    enemy_danger += enemy_danger_2;

    double btn_y_diff = simulation.get_lowest_button_point(my_id) -
                        simulation.get_lowest_button_point(enemy_id);
    double end_game_coef =
            (double) std::min(GAME::TICK_TO_DEADLINE, simulation.sim_tick_index) / GAME::TICK_TO_DEADLINE;

//    if (btn_y_diff < 0 && simulation.cars[my_id]->in_air()) {
//        btn_y_diff = 1.0 - pow(1.0025, -btn_y_diff); //(-11, 11)
//    } else {
//        btn_y_diff = pow(1.0025, btn_y_diff) - 1.0; //(-1:11)
//    }

    btn_y_diff *= end_game_coef * 10.0;

    double my_to_en = simulation.get_my_distance_to_enemy_button_2(my_id, enemy_id);
//    double my_to_en_2 = simulation.get_my_distance_to_enemy_button_2(my_id, enemy_id);
    double ens_to_me = simulation.get_my_distance_to_enemy_button_2(enemy_id, my_id);
    double aim{ens_to_me - 2.0 * my_to_en};

    double my_excess_angle = -abs(simulation.get_car_angle(my_id)) * 1.0;
    double enemy_excess_angle = abs(simulation.get_car_angle(enemy_id)) * 1.0;
    double speed{cpvlength(cpBodyGetVelocity(simulation.cars[my_id]->car_body)) -
                 cpvlength(cpBodyGetVelocity(simulation.cars[enemy_id]->car_body))};

//    if (simulation.sim_tick_index - simulation.cars[my_id]->last_touched < 20) {
//        my_excess_angle *= 750.0;
//    }
//
//    if (simulation.sim_tick_index - simulation.cars[enemy_id]->last_touched < 20) {
//        enemy_excess_angle *= 750.0;
//    }

    if (simulation.get_my_distance_to_enemy_button(enemy_id, my_id) < 42.0) {
        aim *= 0.1;
        speed *= 0.1;
    }

//    aim += (simulation.cars[enemy_id]->dist_to_map() - simulation.cars[my_id]->dist_to_map()) * 50.0;

    double position_on_map{end_game_coef * end_game_coef * simulation.get_position_score(my_id) * 500.0};
    if (simulation.map->external_id == 5) {
        cpVect v1 = cpBodyGetPosition(simulation.cars[my_id]->rear_wheel_body);
        cpVect v2 = cpBodyGetPosition(simulation.cars[my_id]->front_wheel_body);
        if (std::min(v1.x, v2.x) < 100.0 || std::max(v1.x, v2.x) > 1100.0) {
            position_on_map -= 9000000.0;
        }
    }

    if (simulation.map->external_id == 6) {
        cpVect v1 = cpBodyGetPosition(simulation.cars[my_id]->rear_wheel_body);
        cpVect v2 = cpBodyGetPosition(simulation.cars[my_id]->front_wheel_body);
        double y = std::min(v1.y, v2.y);
        if (y < 210.0) {
            position_on_map -= (220.0 - y) * 90000.0;
        }
        speed *= 0.0;
        aim *= 0.0;
    }


    solution.fitness_components[2] += speed * mul;
    solution.fitness_components[3] += my_danger * mul;
    solution.fitness_components[4] += enemy_danger * mul;
    solution.fitness_components[5] += my_excess_angle * mul;
    solution.fitness_components[6] += aim * mul;
    solution.fitness_components[7] += btn_y_diff * mul;
    solution.fitness_components[8] += position_on_map * mul;
    solution.fitness_components[9] += enemy_excess_angle * mul;
}

void
Solver::calcSquareFitness(Simulation &simulation, Solution &solution, int my_id, int enemy_id, double mul,
                          double mul2) {
    double my_danger = simulation.get_closest_point_to_button(my_id);
    double enemy_danger = simulation.get_closest_point_to_button(enemy_id);

    constexpr double dng_threshold = 52.0;
    if (my_danger > dng_threshold) {
        my_danger = dng_threshold;
    }

    if (my_danger < 3.0) {
        solution.fitness_components[9] -= 100000.0;
    }

    my_danger *= 0.15;
    my_danger = my_danger / (1.0 + abs(my_danger));
    my_danger *= 2500;

    if (enemy_danger > dng_threshold) {
        enemy_danger = dng_threshold;
    }
    enemy_danger *= -30.0; // 50 = 300, 75 = 450

    double my_danger_2 = simulation.get_closest_point_to_button2(my_id);
    double enemy_danger_2 = simulation.get_closest_point_to_button2(enemy_id);
    if (my_danger_2 < 3.0) {
        solution.fitness_components[9] -= 100000.0;
    }
    my_danger_2 *= 25;
    enemy_danger_2 *= -20.0; // 50 = 300, 75 = 450
    my_danger += my_danger_2;
    enemy_danger += enemy_danger_2;

    double btn_y_diff = simulation.get_lowest_button_point(my_id) -
                        simulation.get_lowest_button_point(enemy_id);
    double end_game_coef =
            (double) std::min(GAME::TICK_TO_DEADLINE, simulation.sim_tick_index) / GAME::TICK_TO_DEADLINE;

//    if (btn_y_diff < 0 && simulation.cars[my_id]->in_air()) {
//        btn_y_diff = 1.0 - pow(1.0025, -btn_y_diff); //(-11, 11)
//    } else {
//        btn_y_diff = pow(1.0025, btn_y_diff) - 1.0; //(-1:11)
//    }

    btn_y_diff *= end_game_coef * 10.0;


    double my_to_en = simulation.get_my_distance_to_enemy_button_2(my_id, enemy_id);
    double ens_to_me = simulation.get_my_distance_to_enemy_button_2(enemy_id, my_id);
    double aim{ens_to_me - my_to_en};

    if (my_to_en < ens_to_me) {
        aim += 800.0 - simulation.get_my_distance_to_enemy_button(my_id, enemy_id);
    }

    double my_excess_angle = -abs(simulation.get_car_angle(my_id)) * 350.0;
    double enemy_excess_angle = abs(simulation.get_car_angle(enemy_id)) * 350.0;
    double speed{cpvlength(cpBodyGetVelocity(simulation.cars[my_id]->car_body)) -
                 cpvlength(cpBodyGetVelocity(simulation.cars[enemy_id]->car_body))};

    if (simulation.sim_tick_index < 300) {
        my_excess_angle *= 1.5;
    }

    if (simulation.get_my_distance_to_enemy_button(enemy_id, my_id) < 42.0) {
        aim *= 0.1;
        speed *= 0.1;
    } else {
        speed *= 2.5;
    }

    aim += (simulation.cars[enemy_id]->dist_to_map() - simulation.cars[my_id]->dist_to_map()) * 50.0;

    double position_on_map{0};
    if (simulation.map->external_id == 5) {
        cpVect v1 = cpBodyGetPosition(simulation.cars[my_id]->rear_wheel_body);
        cpVect v2 = cpBodyGetPosition(simulation.cars[my_id]->front_wheel_body);
        if (std::min(v1.x, v2.x) < 100.0 || std::max(v1.x, v2.x) > 1100.0) {
            position_on_map -= 9000000.0;
        }
    } else {
    }

    solution.fitness_components[2] += speed * mul;
    solution.fitness_components[3] += my_danger * mul;
    solution.fitness_components[4] += enemy_danger * mul;
    solution.fitness_components[5] += my_excess_angle * mul;
    solution.fitness_components[6] += aim * mul;
    solution.fitness_components[7] += btn_y_diff * mul;
    solution.fitness_components[8] += position_on_map * mul;
    solution.fitness_components[9] += enemy_excess_angle * mul;
}


void
Solver::calcBusFitness(Simulation &simulation, Solution &solution, int my_id, int enemy_id, double mul, double mul2) {

    double my_danger = simulation.get_closest_point_to_button(my_id);
    double enemy_danger = simulation.get_closest_point_to_button(enemy_id, true);

    constexpr double dng_threshold = 72.0;
    if (my_danger > dng_threshold) {
        my_danger = dng_threshold;
    }

    if (my_danger < 3.0) {
        solution.fitness_components[9] -= 100000.0;
    }

    my_danger *= 0.15;
    my_danger = my_danger / (1.0 + abs(my_danger));
    my_danger *= 500;

    if (enemy_danger > dng_threshold) {
        enemy_danger = dng_threshold;
    }
    enemy_danger *= -70.0; // 50 = 300, 75 = 450

    double btn_y_diff = simulation.get_lowest_button_point(my_id) -
                        simulation.get_lowest_button_point(enemy_id);
    double end_game_coef =
            (double) std::min(GAME::TICK_TO_DEADLINE / 2, simulation.sim_tick_index) / GAME::TICK_TO_DEADLINE;
//
//    if (btn_y_diff < 0 && simulation.cars[my_id]->in_air()) {
//        btn_y_diff = 1.0 - pow(1.0025, -btn_y_diff); //(-11, 11)
//    } else {
//        btn_y_diff = pow(1.0025, btn_y_diff) - 1.0; //(-1:11)
//    }
    btn_y_diff *= 3.4;//



    double speed{0.0};

//    if (simulation.sim_tick_index - simulation.cars[my_id]->last_touched < 20) {
//        my_excess_angle *= 750.0;
//    }
//
//    if (simulation.sim_tick_index - simulation.cars[enemy_id]->last_touched < 20) {
//        enemy_excess_angle *= 750.0;
//    }

//    if (simulation.get_my_distance_to_enemy_button(enemy_id, my_id) < 42.0) {


//    }

    double position_on_map{0.0};

//    if (simulation.sim_tick_index > 100) {
        position_on_map +=                (simulation.get_position_score(my_id)) * 1000.0;
//        speed = cpvlength(cpBodyGetVelocity(simulation.cars[my_id]->car_body)) -
//                cpvlength(cpBodyGetVelocity(simulation.cars[enemy_id]->car_body));
//        btn_y_diff += 90.0 * (simulation.get_lowest_bus_point(my_id) - simulation.get_lowest_bus_point(enemy_id));
//    }
//    double position_on_map{0.0};
    if (simulation.map->external_id == 5) {
        cpVect v1 = cpBodyGetPosition(simulation.cars[my_id]->rear_wheel_body);
        cpVect v2 = cpBodyGetPosition(simulation.cars[my_id]->front_wheel_body);
        if (std::min(v1.x, v2.x) < 100.0 || std::max(v1.x, v2.x) > 1100.0) {
            position_on_map -= 9000000.0;
        }
        speed *= 0.0;
    } else if (simulation.map->external_id == 6) {
        cpVect v1 = cpBodyGetPosition(simulation.cars[my_id]->rear_wheel_body);
        cpVect v2 = cpBodyGetPosition(simulation.cars[my_id]->front_wheel_body);
        double y = std::min(v1.y, v2.y);
        if (y < 210.0) {
            position_on_map -= (220.0 - y) * 90000.0;
        }

//        if (std::min(v1.y, v2.y) < 210.0) {
//            position_on_map -= 9000000.0;
//        }

//        double a= cpvdist(v1,cpv(600.0,175.0));
//        double b= cpvdist(v2,cpv(600.0,175.0));
//        constexpr double thr = 170.0;
//        if(a<thr){
//            position_on_map-=4500000.0+4500000.0 *(1.0-a/thr);
//        }
//
//        if(b<thr){
//            position_on_map-=4500000.0+4500000.0 *(1.0-b/thr);
//        }

        if (simulation.sim_tick_index < 600.0) {
//            double my_excess_angle = -std::max(0.0, abs(simulation.get_car_angle(my_id)) - 0.89) * 35000.0;
//
//            position_on_map += my_excess_angle;
//
            if (simulation.sim_tick_index > 100.0) {
                position_on_map += 10000000.0 * simulation.get_lowest_bus_point(my_id).y;
            }
            double a = cpvdist(v1, cpv(600.0, 175.0));
            double b = cpvdist(v2, cpv(600.0, 175.0));
            constexpr double thr = 170.0;
            if (a < thr) {
                position_on_map -= 450.0 + 450.0 * (1.0 - a / thr);
            }

            if (b < thr) {
                position_on_map -= 450.0 + 450.0 * (1.0 - b / thr);
            }
        }
        speed *= 0.0;
//        position_on_map+=std::abs(simulation.get_lowest_bus_point(my_id).x-600.0)*6.35;
    } else if (simulation.map->external_id == 1) {
//        cpVect v1 = cpBodyGetPosition(simulation.cars[my_id]->rear_wheel_body);
//        cpVect v2 = cpBodyGetPosition(simulation.cars[my_id]->front_wheel_body);
//        double a = cpvdist(v1, cpv(600.0, 100.0));
//        double b = cpvdist(v2, cpv(600.0, 100.0));
//        constexpr double thr = 310.0;
//        if (a < thr) {
//            position_on_map -= 4500.0 + 4500.0 * (1.0 - a / thr);
//        }
//
//        if (b < thr) {
//            position_on_map -= 4500.0 + 4500.0 * (1.0 - b / thr);
//        }
        speed *= 0.0;
    }


    solution.fitness_components[2] += speed * mul;
    solution.fitness_components[3] += my_danger * mul;
    solution.fitness_components[4] += enemy_danger * mul;
//    solution.fitness_components[6] += aim * mul;
    solution.fitness_components[7] += btn_y_diff * mul2* mul2* mul2;
    solution.fitness_components[8] += position_on_map * mul2* mul2* mul2;

    //*******************************************************
//    double btn_y_diff = simulation.get_lowest_button_point(my_id) - simulation.get_lowest_button_point(enemy_id);
//    btn_y_diff *= (double) std::min(GAME::TICK_TO_DEADLINE, simulation.sim_tick_index) / GAME::TICK_TO_DEADLINE;
//    btn_y_diff *= 1.0;
//
//    double my_danger = simulation.get_closest_point_to_button(my_id);
//    if (my_danger < 2.5) {
//        solution.fitness_components[9] += -500000 * mul;
//    }
//
//    my_danger *= 0.15;
//    my_danger = my_danger / (1.0 + abs(my_danger));
//    my_danger *= 350;
//
//    double enemy_danger = simulation.get_closest_point_to_button2(enemy_id);
//    enemy_danger *= -0.95;
//
//    double position_on_map = simulation.get_position_score(my_id);
//    position_on_map *= 1000;
//
//    solution.fitness_components[3] += my_danger * mul;
//    solution.fitness_components[4] += enemy_danger * mul;
////    solution.fitness_components[5] += my_angle * mul;
////    solution.fitness_components[6] += aim * mul;
//    solution.fitness_components[7] += btn_y_diff * mul2 * mul2;
//    solution.fitness_components[8] += position_on_map * mul2 * mul2;

// **********************************
//    double min_dist = simulation.get_closest_point_to_button(my_id);
//    fitness += min_dist * mul;
//    min_dist = simulation.get_closest_point_to_button(enemy_id);
//    fitness += -min_dist * 0.9 * mul;
//    fitness += (simulation.get_lowest_button_point(my_id) - simulation.get_lowest_button_point(enemy_id)) * 1.9 * mul;
//    if (simulation.sim_tick_index >= GAME::TICK_TO_DEADLINE) {
//        fitness += std::min(250.0, simulation.get_my_distance_to_enemy_button(enemy_id, my_id)) * mul;
//    } else {
//        fitness += -simulation.get_my_distance_to_enemy_button(my_id, enemy_id) * mul;
//    }
// ***************
// aggressive
// **********************************
//        double min_dist = simulation.get_closest_point_to_button(my_id);
//        fitness += min_dist* 1.0 * mul;
//        min_dist = simulation.get_closest_point_to_button(enemy_id);
//        fitness += -min_dist * 0.9 * mul;
////        fitness += (simulation.get_lowest_button_point(my_id) - simulation.get_lowest_button_point(enemy_id)) * ((double) std::min(GAME::TICK_TO_DEADLINE, simulation.sim_tick_index) / GAME::TICK_TO_DEADLINE) * mul2;
//        if (simulation.sim_tick_index >= GAME::TICK_TO_DEADLINE) {
//            fitness += std::min(250.0, simulation.get_my_distance_to_enemy_button(enemy_id, my_id)) * mul;
//        } else {
//            fitness += -simulation.get_my_distance_to_enemy_button(my_id, enemy_id)*5*((double) std::min(GAME::TICK_TO_DEADLINE, simulation.sim_tick_index) / 300.0) * mul;
//        }
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
