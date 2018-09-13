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

//            if(my_id==enemy_player_id) {
//                for (int i = 1; i < GAME::MOVES_COUNT - 1; i++) {
//                    if (i != previous[0].moves[0]) {
//                        previous[idx].copy_from(best_solutions[my_id]);
//                        previous[idx].moves[0] = i;
//                        evaluate(simulation, previous[idx], best_solutions, my_id, enemy_id);
//                        if (previous[idx].fitness > best->fitness) {
//                            best = &previous[idx];
//                        }
//
//                        idx++;
//                    }
//                }
//            }

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
    double mul(1.0), mul2(0.35);// / 6.8);

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
                fitness -= 9000000.0 * mul;
                mul *= GA::THETA;
                i++;
            }
            break;
        }

        if (!simulation.cars[enemy_id]->alive) {
            while (i < GA::DEPTH) {
                fitness += 5000000.0 * mul;
                mul *= GA::THETA;
                i++;
            }
            break;
        }

//        double aim_distance = simulation.get_my_distance_to_enemy_button(my_id, enemy_id);

        double all_danger = simulation.get_closest_point_to_button(my_id);

        double all_enemy_danger = simulation.get_closest_point_to_button(enemy_id);

        double deadline_distance = simulation.get_distance_to_deadline(my_id)-simulation.get_distance_to_deadline(enemy_id);

        all_danger *= c->h;
        all_danger = all_danger / (1.0 + abs(all_danger));
        fitness += all_danger * c->a * mul;

//        all_enemy_danger /= 20.0;
//        all_enemy_danger = 150.0 * all_enemy_danger / (1.0 + abs(all_enemy_danger));
        fitness -= all_enemy_danger * c->b * mul;

//        fitness += cpvlength(cpBodyGetVelocity(simulation.cars[my_id]->car_body)) * mul;
//
#define PI 3.14159265358979323846264338327950288
        double enemy_angle = cpBodyGetAngle(simulation.cars[enemy_id]->car_body);
        while (enemy_angle > PI) {
            enemy_angle -= 2.0 * PI;
        }
        while (enemy_angle < -PI) {
            enemy_angle += 2.0 * PI;
        }


        fitness += abs(enemy_angle) * c->c * mul;

//        deadline_distance = 100.0 * deadline_distance / (1.0+abs(deadline_distance));
        fitness += deadline_distance * mul2 * c->d;

        if (simulation.sim_tick_index >= GAME::TICK_TO_DEADLINE) {
            fitness += std::min(c->g, simulation.get_my_distance_to_enemy_button( enemy_id, my_id)) * mul * c->e;
        }else{
            fitness -= simulation.get_my_distance_to_enemy_button(my_id, enemy_id) * mul * c->f;
        }

#else
//        0  >>>>  8.66665 SOLUTION [
//                a = -9242.102279542642,
//                b = 5621.871226134015,
//                c = 710.0720963489279,
//                d = 546.888307544767,
//                h = 1.9000258060474517,
//                i = 38.17492490460623,
//                e = 0.999267186061159,
//                f = 0.6465014628975089,


//        a=-4437.245968136879, b=5170.653593041717, c=280.5339818543793, d=462.0302565868644, h=1.8985810957226348, i=18.695256396980756, e=0.5274861618607419, f=0.3472523653585986, mult_1=0: 6.8121
//        if (!simulation.cars[my_id]->alive) {
//            while (i < GA::DEPTH) {
//                fitness += -9000000.0 * mul;
//                mul *= GA::THETA;
//                i++;
//            }
//            break;
//        }
//
//        if (!simulation.cars[enemy_id]->alive) {
//            while (i < GA::DEPTH) {
//                fitness += 5000000.0 * mul;
//                mul *= GA::THETA;
//                i++;
//            }
//            break;
//        }
//
////        double aim_distance = simulation.get_my_distance_to_enemy_button(my_id, enemy_id);
//
//        double all_danger = simulation.get_closest_point_to_button(my_id);
//
//        double all_enemy_danger = simulation.get_closest_point_to_button(enemy_id);
//
//        double deadline_distance = simulation.get_distance_to_deadline(my_id)-simulation.get_distance_to_deadline(enemy_id);
//
//        all_danger /= 2.5;
//        all_danger = 100.0 * all_danger / (1.0 + abs(all_danger));
//        fitness += all_danger * 2.0 * mul;
//
////        all_enemy_danger /= 20.0;
////        all_enemy_danger = 150.0 * all_enemy_danger / (1.0 + abs(all_enemy_danger));
//        fitness -= all_enemy_danger * 2.42 * mul;
//
////        fitness += cpvlength(cpBodyGetVelocity(simulation.cars[my_id]->car_body)) * mul;
////
//#define PI 3.14159265358979323846264338327950288
//        double enemy_angle = cpBodyGetAngle(simulation.cars[enemy_id]->car_body);
//        while (enemy_angle > PI) {
//            enemy_angle -= 2.0 * PI;
//        }
//        while (enemy_angle < -PI) {
//            enemy_angle += 2.0 * PI;
//        }
//
//        fitness += abs(enemy_angle) * 69.0 * mul;
//
////        deadline_distance = 100.0 * deadline_distance / (1.0+abs(deadline_distance));
//        fitness += deadline_distance * mul2 * 1.46;
//
//        if (simulation.sim_tick_index >= GAME::TICK_TO_DEADLINE) {
//            fitness += std::min(100.0, simulation.get_my_distance_to_enemy_button( enemy_id, my_id)) * mul * 1.0;
//        }else{
//            fitness -= simulation.get_my_distance_to_enemy_button(my_id, enemy_id) * mul * 0.675;
//        }


// **********************************

        if (!simulation.cars[my_id]->alive) {
            while (i < GA::DEPTH) {
                fitness += -9000.0 * mul;
                mul *= GA::THETA;
                i++;
            }
            break;
        }

        if (!simulation.cars[enemy_id]->alive) {
            while (i < GA::DEPTH) {
                fitness += 8000.0 * mul;
                mul *= GA::THETA;
                i++;
            }
            break;
        }

        double min_dist = simulation.get_closest_point_to_button(my_id);
        fitness += min_dist * mul;

        min_dist = simulation.get_closest_point_to_button(enemy_id);
        fitness += -min_dist * 0.9 * mul;

        fitness += (simulation.get_distance_to_deadline(my_id) - simulation.get_distance_to_deadline(enemy_id)) * 1.9 * mul;
        if (simulation.sim_tick_index >= GAME::TICK_TO_DEADLINE) {
            fitness += std::min(50.0, simulation.get_my_distance_to_enemy_button(enemy_id, my_id)) * mul;
        } else {
            fitness += -simulation.get_my_distance_to_enemy_button(my_id, enemy_id) * mul;
        }
#endif

        mul *= GA::THETA;
//        if (simulation.sim_tick_index - simulation.saved_tick > GA::DEPTH * 0.66) {
//            mul *= GA::THETA;
//        }
//        if (simulation.sim_tick_index - simulation.saved_tick > GA::DEPTH * 0.75) {
//            mul *= GA::THETA;
//        }

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
