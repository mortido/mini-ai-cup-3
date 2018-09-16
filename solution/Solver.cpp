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
    double mul(1.0), mul2(0.35);//007);
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
                fitness += -9000000000.0 * mul;
                mul *= GA::THETA;
                i++;
            }
            break;
        }

        if (!simulation.cars[enemy_id]->alive) {
            fitness += 1000000000.0 * mul;
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
            fitness += calcFitness(simulation, test_solution, my_id, enemy_id, mul, mul2);
        }

        mul *= GA::THETA;
        mul2 *= GA::THETA_PLUS;
//        mul2 *= GA::THETA_PLUS;
//        mul2 *= GA::THETA_PLUS;
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

double
Solver::calcFitness(Simulation &simulation, Solution &solution, int my_id, int enemy_id, double mul, double mul2) {
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

//            (FloatValue, "a", 7.441622597558529)
//            (FloatValue, "b", 1.5218038846779693)
//            (FloatValue, "c", 0.005589912179041602)
//            (FloatValue, "d", 765.0537256940278)
//            (FloatValue, "e", 4.55941457810174)
//            (FloatValue, "f", 27.015904156136347)
//            (FloatValue, "g", 46.10340056196941)
//            (FloatValue, "h", 101.79234387829027)

    double my_danger = simulation.get_closest_point_to_button(my_id);
    if (my_danger < 5.0) {
        return -500000000.0 * mul;
    }

    double enemy_danger = simulation.get_closest_point_to_button2(enemy_id);
    my_danger *= 0.07;
    my_danger = my_danger / (1.0 + abs(my_danger));
    my_danger *= 900;
//    my_danger *= 5.0;
    enemy_danger *= -1.0;


    double btn_y_diff = simulation.get_lowest_button_point(my_id) -
                        simulation.get_lowest_button_point(enemy_id);
    double end_game_coef = (double) simulation.sim_tick_index / GAME::TICK_TO_DEADLINE;
    btn_y_diff *= end_game_coef;
    btn_y_diff *= 1.0;

    double my_to_en = simulation.get_my_distance_to_enemy_button(my_id, enemy_id);
    double ens_to_me = simulation.get_my_distance_to_enemy_button(enemy_id, my_id);
//    double positioning = -my_to_en + ens_to_me;
    double positioning = ens_to_me / my_to_en;

    double enemy_angle = 0*abs(simulation.get_car_angle(enemy_id));
    double my_angle = 0*abs(simulation.get_car_angle(my_id));
    cpVect my_pos = cpBodyLocalToWorld(simulation.cars[my_id]->car_body,
                                       cpBodyGetCenterOfGravity(simulation.cars[my_id]->car_body));
    cpVect enemy_pos = cpBodyLocalToWorld(simulation.cars[enemy_id]->car_body,
                                          cpBodyGetCenterOfGravity(simulation.cars[enemy_id]->car_body));

//    fitness += cpvlength(cpBodyGetVelocity(simulation.cars[my_id]->car_body)) * mul * 2.5;

    double aim{my_to_en * -positioning};
//    if (positioning > 0.98){//} && (my_pos.y - enemy_pos.y) > -42.0) {
//        aim = 200.0+ my_to_en * -1.5;
//        aim *= (1.0 - end_game_coef);
//    } else {
//        aim = abs(my_pos.x - enemy_pos.x) * 1.4;
//    };

//    if((my_pos.y - enemy_pos.y) > -42.0) {
//        positioning *= 20.0;
//    } else {
//        positioning*=5.0;
//    }

    positioning*=800.0;
//
//    if(enemy_angle>PI/2.0) {
//        enemy_angle = enemy_angle * 5000.0;// + my_angle * abs(cpBodyGetAngularVelocity(simulation.cars[my_id]->car_body));
//    }
//    enemy_angle *= 100.0;
//    my_angle = my_angle * -666.0 * 0.0;

//    my_angle = abs(my_angle);
//    if (my_angle < PI / 2.0) {
//        my_angle = 0;
//    } else {
//        my_angle *= 46.1;
//    }

    fitness += (my_danger + enemy_danger + positioning) * mul + (btn_y_diff + aim + my_angle + enemy_angle) * mul2;

//    fitness += mul2 * simulation.get_position_score(my_id) * 106.6;

#ifdef REWIND_VIEWER
    if (print_fitness) {
        simulation.rewind.message("%f %f %fd %fd %f %f\\n", my_danger, enemy_danger,
                                  enemy_angle, my_angle, aim, btn_y_diff);
//        simulation.rewind.message("%f \\n",
//                                  (my_btn_dist_to_objects - enemy_btn_dist_to_objects - my_angle + enemy_angle + aim) *
//                                  mul + (btn_y_diff) * mul2);
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

//#ifdef OPTIMIZATION_RUN
//#else
//
//    double my_btn_dist_to_objects = simulation.get_closest_point_to_button(my_id);
//    double enemy_btn_dist_to_objects = simulation.get_closest_point_to_button(enemy_id);
//    double btn_y_diff = simulation.get_lowest_button_point(my_id) -
//                        simulation.get_lowest_button_point(enemy_id);
//    double position_on_map = simulation.get_position_score(my_id);
//
//
//    btn_y_diff *= (double) std::min(GAME::TICK_TO_DEADLINE, simulation.sim_tick_index) / GAME::TICK_TO_DEADLINE;
//    btn_y_diff *= 16.8;
//
//    my_btn_dist_to_objects *= 0.02;
//    my_btn_dist_to_objects = my_btn_dist_to_objects / (1.0 + abs(my_btn_dist_to_objects));
//    my_btn_dist_to_objects *= 100;
//
//    enemy_btn_dist_to_objects *= 1.;
//
//    position_on_map *= 1000;
//
//    fitness += (my_btn_dist_to_objects - enemy_btn_dist_to_objects) * mul + (btn_y_diff + position_on_map) * mul2;
//
//#ifdef REWIND_VIEWER
//    if (print_fitness) {
//        simulation.rewind.message("me_d: %f en_d: %f pos:%f diff:%f\\n", my_btn_dist_to_objects,
//                                  enemy_btn_dist_to_objects,
//                                  position_on_map, btn_y_diff);
//        simulation.rewind.message("BUS: %f \\n",
//                                  (my_btn_dist_to_objects - enemy_btn_dist_to_objects) * mul +
//                                  (btn_y_diff + position_on_map) * mul2);
//        print_fitness = false;
//    }
//#endif
// **********************************
    double min_dist = simulation.get_closest_point_to_button(my_id);
    fitness += min_dist * mul;
    min_dist = simulation.get_closest_point_to_button(enemy_id);
    fitness += -min_dist * 0.9 * mul;
    fitness += (simulation.get_lowest_button_point(my_id) - simulation.get_lowest_button_point(enemy_id)) * 1.9 * mul;
    if (simulation.sim_tick_index >= GAME::TICK_TO_DEADLINE) {
        fitness += std::min(250.0, simulation.get_my_distance_to_enemy_button(enemy_id, my_id)) * mul;
    } else {
        fitness += -simulation.get_my_distance_to_enemy_button(my_id, enemy_id) * mul;
    }
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
//#endif
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
