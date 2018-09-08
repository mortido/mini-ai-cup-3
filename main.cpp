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

#ifdef LOCAL_RUN

#include <fstream>

#endif

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

int main(int argc, char *argv[]) {
    Simulation simulation;
    Solver solver;

    double time_bank = 0.0 * 120.0;
    int round{-1}, tick_index{0}, global_tick_index{0};

    int my_player_id{0}, enemy_player_id{1};
    int my_lives, enemy_lives;

    string input_string, input_type;
    high_resolution_clock::time_point start_time;

    array<enemyPrediction, GAME::MOVES_COUNT> enemyPredictions{};
    int my_prev_move{0}, my_prev_prev_move{0};


#ifdef LOCAL_RUN
    time_bank = time_bank * 17.0 / 9.0;
//    std::fstream file;
//    if (argc == 1) {
//        file.open("data.json", std::ios::out);
//    } else {
//        file.open(argv[1], std::ios::in);
//    }
#endif
    while (true) {
#ifdef LOCAL_RUN
        if (argc == 1) {
            getline(cin, input_string);
//            file << input_string << std::endl;
//            file.close();
//            file.open("data.json", std::ios::out);
        } else {
//            getline(file, input_string);
//            if (file.eof()) {
//                break;
//            }
        }
#else
        getline(cin, input_string);
#endif
        start_time = NOW;
        if (!input_string.length()) {
            break;
        }

        auto state = nlohmann::json::parse(input_string);
        input_type = state["type"].get<std::string>();
        auto params = state["params"];

        if (input_type == "new_match") {
            round++;
            tick_index = 0;
            my_lives = params["my_lives"].get<int>();
            enemy_lives = params["enemy_lives"].get<int>();
            solver.new_round(time_bank, my_lives, enemy_lives);

#ifdef LOCAL_RUN
            if (round) {
                cerr << "round " << round << " pos_diff_sum: " << simulation.car_pos_error.x << " ; ";
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
        } else if (input_type == "tick") {

            solver.new_tick(tick_index, my_prev_move);
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
                for (int m = 0; m < GAME::MOVES_COUNT; m++) {
                    double err = get_predict_error(enemyPredictions[m], params["enemy_car"]);
                    if (err < min_error) {
                        min_error = err;
                        enemy_prev_prev = m - 1;
                    }
                }

                // catch up with "reality" - 1 tick
                simulation.restore();
                simulation.move_car(my_player_id, my_prev_prev_move);
                simulation.move_car(enemy_player_id, enemy_prev_prev);
                simulation.step();
            }

            simulation.save();

            if (tick_index) {
                // simulate 3 enemy moves and store them for later prediction
                for (int m = 0; m < GAME::MOVES_COUNT; m++) {
                    simulation.restore();
                    simulation.move_car(my_player_id, my_prev_move);
                    simulation.move_car(enemy_player_id, m - 1);
                    simulation.step();
                    simulation.step(); // commands doesn't matters for second step.
                    enemyPredictions[m].car_pos = cpBodyGetPosition(simulation.cars[enemy_player_id]->car_body);
                    enemyPredictions[m].rear_wheel_pos = cpBodyGetPosition(
                            simulation.cars[enemy_player_id]->rear_wheel_body);
                    enemyPredictions[m].front_wheel_pos = cpBodyGetPosition(
                            simulation.cars[enemy_player_id]->front_wheel_body);
                }
            }
            cerr << "---------------------" << endl;
            my_prev_prev_move = my_prev_move;
            cerr << "---------------------" << endl;

//            solver.solve(simulation,start_time,my_player_id, enemy_player_id);
            cerr << "---------------------" << endl;

            my_prev_move = solver.best_solutions[my_player_id].moves[0];
            cerr << "---------------------" << endl;

            cout << solver.best_solutions[my_player_id].to_json(tick_index > 0 ? 1: 0).dump() << endl;
            cerr << "---------------------" << endl;
            tick_index++;
            global_tick_index++;
            time_bank -= ELAPSED_TIME;
            cerr << "---------------------" << endl;
#ifdef LOCAL_RUN
            simulation.restore();
            if (tick_index) {
                simulation.step();
            }
            simulation.check(my_player_id, params);
            cerr << "---------------------" << endl;
#ifdef REWIND_VIEWER
            simulation.rewind.message("SIMS(%d): %d\\n", GA::DEPTH, solver.simulations);
            simulation.draw(params, my_player_id);
#endif
#endif
        } else {
            break;
        }
    }

    return 0;
}
