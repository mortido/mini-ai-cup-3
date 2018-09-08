#include <iostream>
#include <chrono>
#include <array>
#include <chipmunk/chipmunk.h>
#include "../nlohmann/json.hpp"

#include "random/Randomizer.h"
#include "simulation/Simulation.h"
#include "Solution.h"
#include "Solver.h"

using namespace std;
using namespace std::chrono;

#ifdef REWIND_VIEWER

#include "RewindClient.h"

#endif

#define NOW high_resolution_clock::now()
#define ELAPSED_TIME duration_cast<duration<double>>(NOW - start_time).count()

//int main() {
//    double time_bank = 0.0 * 120.0;
//
//#ifdef LOCAL_RUN
//    time_bank = time_bank * 17.0 / 9.0;
//#endif
//
//    // make preparations
//    Solver solver;
//    Simulation simulation;
//    Randomizer::init();
//
//    high_resolution_clock::time_point start_time;
//    string input_string, input_type;
//
//    cpFloat preva = 0.0, prevb = 0.0;
//
//    while (true) {
//        getline(cin, input_string);
//        start_time = NOW;
//
//        // parse inputs
//        auto state = nlohmann::json::parse(input_string);
//        input_type = state["type"].get<std::string>();
//
//        if (input_type == "new_match") {
//            // init game constants
////            GameConstants::initConstants(state["params"]);
//            simulation.new_round(state["params"]);
//            solver.time_limit =
//                    time_bank / ((simulation.my_lives + simulation.enemy_lives - 1) * GAME::MAX_ROUND_TICKS);
//
//
////            cerr << "TIME LIMIT IS " << solver.time_limit << " "
//              cerr << "NEW ROUND, wheels is squared ="   << state["params"]["proto_car"].value("squared_wheels", false) << endl;
//        } else if (input_type == "tick") {
//
//#ifdef REWIND_VIEWER
//            double a = cpBodyGetPosition(simulation.cars[simulation.my_player_id]->car_body).x -
//                       state["params"]["my_car"][0][0].get<cpFloat>();
//            double b = cpBodyGetPosition(simulation.cars[simulation.my_player_id]->car_body).y -
//                       state["params"]["my_car"][0][1].get<cpFloat>();
////            if (abs(a) > 1e-6 || abs(b) > 1e-6) {
//                cerr << "tick " << simulation.tick_index << " pos_diff: " << a << " ; " << b << endl;
////            }
////            cerr << std::setprecision(16) << std::fixed;
////            cerr << "tick" << simulation.tick_index + 1 << ":\tpos_real( ";
////            cerr << state["params"]["my_car"][0][0].get<cpFloat>() << "; ";
////            cerr << state["params"]["my_car"][0][1].get<cpFloat>() << " ), pos_in_simulation ( ";
////            cerr << cpBodyGetPosition(simulation.cars[simulation.my_player_id]->car_body).x << "; ";
////            cerr << cpBodyGetPosition(simulation.cars[simulation.my_player_id]->car_body).y << " )" << endl;
////                 << " ), vel_in_simulation ( ";
////            cerr << cpBodyGetVelocity(simulation.cars[simulation.my_player_id]->car_body).x << "; ";
////            cerr << cpBodyGetVelocity(simulation.cars[simulation.my_player_id]->car_body).y
////                 << " ), delta_pos_real_div_dt ( ";
////            cerr << (state["params"]["my_car"][0][0].get<cpFloat>() - preva) / 0.016 << "; ";
////            cerr << (state["params"]["my_car"][0][1].get<cpFloat>() - prevb) / 0.016 << " )" << endl;
////            preva = state["params"]["my_car"][0][0].get<cpFloat>();
////            prevb = state["params"]["my_car"][0][1].get<cpFloat>();
//
//            simulation.rewind.message("%d - ", simulation.tick_index);
//            simulation.rewind.message("%f - ", cpBodyGetPosition(simulation.cars[simulation.my_player_id]->car_body).x -
//                                               state["params"]["my_car"][0][0].get<cpFloat>());
//            simulation.rewind.message("%f    ",
//                                      cpBodyGetPosition(simulation.cars[simulation.my_player_id]->car_body).y -
//                                      state["params"]["my_car"][0][1].get<cpFloat>());
//            simulation.draw(state["params"]);
//#endif
//
////            simulation.update_tick(state["params"]);
//
////            cerr << "TIME BEFORE SOLVE " << ELAPSED_TIME << endl;
////            solver.solve(simulation, start_time);
////            cerr << "TIME AFTER SOLVE " << ELAPSED_TIME << endl;
//
//
//            int move = solver.best_solutions[simulation.my_player_id].moves[0];
//            if (simulation.tick_index < 200) {
//                move = 0;
//            } else {
//                move = 1;
//            }
//            solver.best_solutions[simulation.my_player_id].moves[0] = move;
//            // write best solution
//            cout << solver.best_solutions[simulation.my_player_id].to_json().dump() << endl;
////            cerr << "round " << simulation.round << " tick " << simulation.tick_index << " simulations "
////                 << solver.simulations
////                 << endl;
////            cerr << solver.best_solutions[simulation.my_player_id].fitness << endl;
////            cerr << solver.best_solutions[simulation.my_player_id].moves[0] << endl;
//
//            simulation.cars[simulation.my_player_id]->move(solver.best_solutions[simulation.my_player_id].moves[0]);
//            simulation.cars[simulation.enemy_player_id]->move(0);
//        } else {
//            cerr << input_type << endl;
//            break;
//        }
//
//        time_bank -= ELAPSED_TIME;
//#ifdef LOCAL_RUN
//        if (input_type == "tick") {
////            simulation.simulate_tick();
////            simulation.simulate_tick();
////            simulation.simulate_tick();
////            simulation.simulate_tick();
////            simulation.simulate_tick();
////            simulation.simulate_tick();
////            simulation.simulate_tick();
////            simulation.simulate_tick();
////            simulation.simulate_tick();
////            simulation.simulate_tick();
////            simulation.simulate_tick();
////            simulation.reset();
//            simulation.simulate_tick();
//        }
//#endif
//    }
//
//    return 0;
//}

// ********************************************************************************************************************
//extern "C" {
//#include <chipmunk/dmm.h>
//}

int main() {
    Simulation simulation;

    int round{-1}, tick_index{0}, global_tick_index{0};
    int my_player_id{0}, enemy_player_id{1};
    int my_lives, enemy_lives;

    string input_string, input_type;
    high_resolution_clock::time_point start_time;

    array<enemyPrediction, GAME::MOVES_COUNT> enemyPredictions{};
    int my_prev_move{0}, my_prev_prev_move{0};
#ifdef LOCAL_RUN
    json prev_params;
#endif
    while (true) {
        getline(cin, input_string);
        if (!input_string.length()) {
            break;
        }

        auto state = nlohmann::json::parse(input_string);
        input_type = state["type"].get<std::string>();
        auto params = state["params"];

        if (input_type == "new_match") {
            round++;
            tick_index = 0;

#ifdef LOCAL_RUN
            if (round) {
                cerr << "round " << round << " car_pos_diff_sum: " << simulation.car_pos_error.x << " ; ";
                cerr << simulation.car_pos_error.y;
                cerr << " squared =" << state["params"]["proto_car"].value("squared_wheels", false) << endl;
                cerr << "BYTES TO COPY " << getBytesToCopy() << endl;
//                cerr << "1000 cycles of (restore)" << endl;
//
//                start_time = NOW;
//                for(int i=0;i<1000;i++){
//                    for(int j=0;j<50;j++) {
//                        simulation.step();
//                    }
////                    simulation.step();
//                    simulation.restore();
//                }
//
//                cerr << "time = " << ELAPSED_TIME << endl;
            }
#endif

            // Init round objects
            simulation.new_round(params);
            my_lives = params["my_lives"].get<int>();
            enemy_lives = params["enemy_lives"].get<int>();
        } else if (input_type == "tick") {
            if (global_tick_index == 0) {
                // init player positions once per game
                if (params["my_car"][2].get<int>() == 1) {
                    my_player_id = 0;
                    enemy_player_id = 1;
                } else {
                    my_player_id = 1;
                    enemy_player_id = 0;
                }
            }

            if (tick_index > 1) {
                // try to gues enemy move by his/her new position
                int enemy_prev_prev = 0;
                double min_error = 999999999.0;
//                cerr << gold_standart.sim_tick_index << "----" << endl;
                for (int m = 0; m < GAME::MOVES_COUNT; m++) {
                    double err = get_predict_error(enemyPredictions[m], params["enemy_car"]);
                    if (err < min_error) {
                        min_error = err;
                        enemy_prev_prev = m - 1;
                    }

//                    cerr << enemyPredictions[m].car_pos.x << ';' << enemyPredictions[m].car_pos.y << endl;
//                    cerr << err << endl;
                }
//                cerr << min_error << endl;
//                cerr << enemy_prev_prev << endl;


                // catch up with "reality" - 1 tick
                simulation.restore();
                simulation.move_car(my_player_id, my_prev_prev_move);
                simulation.move_car(enemy_player_id, enemy_prev_prev);
                simulation.step();
#ifdef LOCAL_RUN
                simulation.check(my_player_id, prev_params);
//                dirty_sim.reset();
//                dirty_sim.step();
//                dirty_sim.check(my_player_id, params);
#endif
            }

            simulation.save();

            if (tick_index) {
                for (int m = 0; m < GAME::MOVES_COUNT; m++) {
                    simulation.restore();
                    simulation.move_car(my_player_id, my_prev_move);
                    simulation.move_car(enemy_player_id, m - 1);
                    simulation.step();
                    simulation.step(); // commands doesn't matters for second step.
//
                    enemyPredictions[m].car_pos = cpBodyGetPosition(simulation.cars[enemy_player_id]->car_body);
                    enemyPredictions[m].rear_wheel_pos = cpBodyGetPosition(
                            simulation.cars[enemy_player_id]->rear_wheel_body);
                    enemyPredictions[m].front_wheel_pos = cpBodyGetPosition(
                            simulation.cars[enemy_player_id]->front_wheel_body);
                }

#ifdef REWIND_VIEWER
                simulation.restore();
                simulation.draw(prev_params);
#endif
            }

#ifdef LOCAL_RUN
            prev_params = params;
#endif
            my_prev_prev_move = my_prev_move;
            if (tick_index < 200) {
                my_prev_move = 0;
            } else {
                my_prev_move = 1;
            }
            json command;
            switch (my_prev_move) {
                case 0:
                    command["command"] = "stop";
                    break;
                case -1:
                    command["command"] = "left";
                    break;
                case 1:
                    command["command"] = "right";
                    break;
                default:
                    command["command"] = "stop";
            }
            cout << command.dump() << endl;

            tick_index++;
            global_tick_index++;
        } else {
            break;
        }
    }

    return 0;
}

//source ./miniaicups/madcars/Runners/local-runner-venv/bin/activate