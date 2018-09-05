//#include <stdio.h>
#include <iostream>
#include <chrono>

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

int main() {
    Simulation gold_standart;
    Simulation dirty_sim;
    int round{-1}, tick_index{0}, global_tick_index{0};
    int my_player_id, enemy_player_id;
    int my_lives, enemy_lives;
    string input_string, input_type;

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
                cerr << "round " << round << " pos_diff_sum: " << gold_standart.car_pos_error.x << " ; ";
                cerr << gold_standart.car_pos_error.y;
                cerr << " squared =" << state["params"]["proto_car"].value("squared_wheels", false) << endl;
                cerr << "dirty_pos_diff_sum: " << dirty_sim.car_pos_error.x << " ; ";
                cerr << dirty_sim.car_pos_error.y << endl;
            }
#endif

            // Init round objects
            gold_standart.new_round(params);
            dirty_sim.new_round(params);

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



            if (tick_index) {
                // try to gues enemy move by his/her new position
                gold_standart.move_car(enemy_player_id, 0);



                dirty_sim.check(my_player_id, params);

                gold_standart.map->draw(gold_standart.rewind);
                gold_standart.draw(params);
                dirty_sim.draw(params);
                dirty_sim.rewind.end_frame();

                // catch up with "reality"
                gold_standart.step();
                dirty_sim.copy_from(gold_standart);

                dirty_sim.step();
                dirty_sim.step();
                dirty_sim.step();
                dirty_sim.step();
                dirty_sim.step();
                dirty_sim.step();
                dirty_sim.step();
                dirty_sim.step();
                dirty_sim.step();
                dirty_sim.step();
                dirty_sim.reset();
            }

#ifdef LOCAL_RUN
            gold_standart.check(my_player_id, params);
#endif
#ifdef REWIND_VIEWER
//
#endif

            int move(0);
            if (tick_index < 200) {
                move = 0;
            } else {
                move = 1;
            }
            json command;
            switch (move) {
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
            gold_standart.move_car(my_player_id, move);

//            dirty_sim.move_car(my_player_id, move);
//            dirty_sim.move_car(enemy_player_id, 0);
//            dirty_sim.step();

            tick_index++;
            global_tick_index++;
        } else {
            break;
        }
    }

    return 0;
}

// ********************************************************************************************************************
//int main() {
//    double time_bank = 0.0*120.0;
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
//            cerr << "TIME LIMIT IS " << solver.time_limit << endl;
//        } else if (input_type == "tick") {
//            simulation.update_tick(state["params"]);
//            if (simulation.tick_index == 0) {
//                solver.init(simulation);
//            }
////            cerr << "TIME BEFORE SOLVE " << ELAPSED_TIME << endl;
//            // find best move
//            solver.solve(simulation, start_time);
////            cerr << "TIME AFTER SOLVE " << ELAPSED_TIME << endl;
//
////            solver.best_solutions[simulation.my_player_id].moves[0] = 0;
////            if(simulation.tick_index==50){
////                solver.best_solutions[simulation.my_player_id].moves[0] = 1;
////            }
//            // write best solution
//            cout << solver.best_solutions[simulation.my_player_id].to_json().dump() << endl;
////            cerr << "round " << simulation.round << " tick " << simulation.tick_index << " simulations "
////                 << solver.simulations
////                 << endl;
////            cerr << solver.best_solutions[simulation.my_player_id].fitness << endl;
////            cerr << solver.best_solutions[simulation.my_player_id].moves[0] << endl;
//
//            cerr << simulation.tick_index << ":";
//            cerr << cpBodyGetPosition(simulation.cars[simulation.my_player_id]->car_body).x - state["params"]["my_car"][0][0].get<cpFloat>() << "; ";
//            cerr << cpBodyGetPosition(simulation.cars[simulation.my_player_id]->car_body).y - state["params"]["my_car"][0][1].get<cpFloat>() << endl;
//
//            simulation.cars[simulation.my_player_id]->move(solver.best_solutions[simulation.my_player_id].moves[0]);
//            simulation.cars[simulation.enemy_player_id]->move(0);
//        } else {
//            cerr << input_type << endl;
//            break;
//        }
//
//        time_bank -= ELAPSED_TIME;
//#ifdef REWIND_VIEWER
//        if (input_type == "tick") {
//            simulation.rewind.message("%d - ", simulation.tick_index);
//            simulation.rewind.message("%f - ", cpBodyGetPosition(simulation.cars[simulation.my_player_id]->car_body).x - state["params"]["my_car"][0][0].get<cpFloat>());
//            simulation.rewind.message("%f", cpBodyGetPosition(simulation.cars[simulation.my_player_id]->car_body).y - state["params"]["my_car"][0][1].get<cpFloat>());
//            simulation.draw();
//            simulation.simulate_tick();
//        }
//#endif
//    }
//
//    return 0;
//}

//*********************************************************************************************************************
//int main() {
//    double time_bank = 120.0;
//
//#ifdef LOCAL_RUN
//    time_bank = time_bank * 17.0 / 9.0;
//#endif
//#ifdef REWIND_VIEWER
//
//    auto &rewind = RewindClient::instance();
//
//#endif
//
//    high_resolution_clock::time_point start_time;
//    string input_string, input_type;
//
//    Simulation simulation;
////    cpSpace *space = cpSpaceNew();
////    cpSpaceSetGravity(space, GAME::GRAVITY);
////    std::unique_ptr<Map> map;
//    std::unique_ptr<Car> car;
//    json proto_car;
//    cerr << std::setprecision(16) << std::fixed;
//    int tick{0};
//    int round(-1);
//    while (true) {
//        getline(cin, input_string);
//        start_time = NOW;
//
//        // parse inputs
//        auto state = nlohmann::json::parse(input_string);
//        input_type = state["type"].get<std::string>();
//
//        if (input_type == "new_match") {
//            round++;
//            cerr << "round" << endl;
//            auto params = state["params"];
//            tick = 0;
//
//            if (round == 0) {
//                simulation.map = std::unique_ptr<Map>(new Map(params["proto_map"], simulation.space));
//                proto_car = params["proto_car"];
//                car = unique_ptr<Car>(new Car(proto_car, simulation.space, 1.0, 0));
//            } else {
//                break;
//            }
//        } else if (input_type == "tick") {
//            auto params = state["params"];
//
//            if (tick == 0) {
//                params["my_car"];
//                // 0 - pos
//                // 1 - angle
//                // 2 - mirror
//                // 3 - front wheel
//                // 4 - rear wheel
////                car = unique_ptr<Car>(new Car(proto_car, simulation.space, params["my_car"][2].get<int>(), 1));
//                car->set_from_json(params["my_car"]);
////                cpBodySetPosition(car->car_body,
////                                  cpv(params["my_car"][0][0].get<cpFloat>(), params["my_car"][0][1].get<cpFloat>()));
////                cpBodySetPosition(car->front_wheel_body,
////                                  cpv(params["my_car"][4][0].get<cpFloat>(), params["my_car"][4][1].get<cpFloat>()));
////                cpBodySetPosition(car->rear_wheel_body,
////                                  cpv(params["my_car"][3][0].get<cpFloat>(), params["my_car"][3][1].get<cpFloat>()));
//
//
//            }
////            cerr << tick << ":" << cpBodyGetPosition(car->car_body).x << "; " << cpBodyGetPosition(car->car_body).y  << "|";
////            cerr << cpBodyGetPosition(car->front_wheel_body).x << "; " << cpBodyGetPosition(car->front_wheel_body).y << "|";
////            cerr << cpBodyGetPosition(car->rear_wheel_body).x << "; " << cpBodyGetPosition(car->rear_wheel_body).y  << endl;
////
////            cerr << tick << ":" << params["my_car"][0][0].get<cpFloat>() << "; " << params["my_car"][0][1].get<cpFloat>() << "|";
////            cerr << params["my_car"][4][0].get<cpFloat>() << "; " << params["my_car"][4][1].get<cpFloat>() << "|";
////            cerr << params["my_car"][3][0].get<cpFloat>() << "; " << params["my_car"][3][1].get<cpFloat>() << endl;
////            cerr << "TIME BEFORE SOLVE " << ELAPSED_TIME << endl;
//
//            cerr << tick << ":";
//            cerr << cpBodyGetPosition(car->car_body).x - params["my_car"][0][0].get<cpFloat>() << "; ";
//            cerr << cpBodyGetPosition(car->car_body).y - params["my_car"][0][1].get<cpFloat>() << endl;
//
//            if (tick < 600) {
//                cout << R"({"command":"stop"})" << endl;
//                car->move(0);
////                for (auto *motor:car->engines) {
////                    cpSimpleMotorSetRate(motor, 0.0);
////                }
//            } else if (tick < 700) {
//                cout << R"({"command":"left"})" << endl;
//                car->move(-1);
////                for (auto *motor:car->engines) {
////                    cpSimpleMotorSetRate(motor, car->max_speed);
////                }
//            } else {
//                break;
//            }
//            cpSpaceStep(simulation.space, GAME::SIMULATION_DT);
//            tick++;
//
//#ifdef REWIND_VIEWER
//            car->draw(rewind);
//            simulation.map->draw(rewind);
//            rewind.end_frame();
//#endif
//        } else {
//            break;
//        }
//        time_bank -= ELAPSED_TIME;
//    }
////    cpSpaceFree(space);
//    cerr << "BYE!" << endl;
//    return 0;
//}