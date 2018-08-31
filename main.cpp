////#include <stdio.h>
//#include <iostream>
//#include <chrono>
//
//#include "chipmunk/include/chipmunk.h"
//#include "../nlohmann/json.hpp"
//
//#include "random/Randomizer.h"
//#include "simulation/Simulation.h"
//#include "Solution.h"
//#include "Solver.h"
//
//using namespace std;
//using namespace std::chrono;
//
//#define NOW high_resolution_clock::now()
//#define ELAPSED_TIME duration_cast<duration<double>>(NOW - start_time).count()
//
//int main() {
//    double time_bank = 120.0;
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
//            solver.time_limit = time_bank / ((simulation.my_lives + simulation.enemy_lives - 1) * GAME::MAX_ROUND_TICKS);
//
//
//            cerr << "TIME LIMIT IS " << solver.time_limit << endl;
//        } else if (input_type == "tick") {
//            simulation.update_tick(state["params"]);
//            cerr << "TIME BEFORE SOLVE " << ELAPSED_TIME << endl;
//            // find best move
//            solver.solve(simulation, start_time);
//            cerr << "TIME AFTER SOLVE " << ELAPSED_TIME << endl;
//            // TODO: simulate all possible enemy positions.
//
//            // write best solution
//            cout << solver.best_solutions[simulation.my_player_id].to_json().dump() << endl;
//            cerr << "round " << simulation.round << " tick " << simulation.tick_index << " simulations " << solver.simulations
//                 << endl;
//            cerr << solver.best_solutions[simulation.my_player_id].fitness << endl;
//            cerr << solver.best_solutions[simulation.my_player_id].moves[0] << endl;
//
//#ifdef REWIND_VIEWER
//            // TODO: simulate best_solutions again and draw them.
//#endif
//        } else {
//            cerr << input_type << endl;
//            // TODO: remove trash line because we are good
//            break;
//        }
//
//        time_bank -= ELAPSED_TIME;
//    }
//
//    return 0;
//}

#include <iostream>

#include "../nlohmann/json.hpp"

using namespace std;

#define PI 3.14159265358979323846264338327950288

int main() {
    string input_string, input_type;
    int tick(0);
    int round(-1);
    while (true) {

        getline(cin, input_string);
        auto state = nlohmann::json::parse(input_string);
        // also possible:
        // json state;
        // cin >> state;

        input_type = state["type"].get<std::string>();
        auto params = state["params"];

        if (input_type == "new_match") {
            int my_lives = params["my_lives"].get<int>();
            int enemy_lives = params["enemy_lives"].get<int>();

            // Example of proto_map parsing.
            auto map = params["proto_map"];
            int map_id = map["external_id"].get<int>();
            auto segments = map["segments"];
            for(auto segment:segments){
                auto fp = segment[0];
                auto sp = segment[1];
                double height = segment[2].get<double>();
            }

            auto proto_car = params["proto_car"];
            // etc...

            round++;
            tick = 0;

            cerr << "Round " << round + 1 << " started!" << endl;

        } else if (input_type == "tick") {
            cerr << "Round " << round + 1;
            cerr << " tick " << tick << endl;

            nlohmann::json command;
            if (tick < 20) {
                // wait a little bit to fall on floor
                command["command"] = "stop";
                cerr << "Waiting..." << endl;
            } else {
                auto my_car = params["my_car"];
                auto enemy_car = params["enemy_car"];

                auto my_pos = my_car[0];
                auto enemy_pos = enemy_car[0];

                // check my and enemy position and go to the enemy
                if(my_pos[0].get<double>() > enemy_pos[0].get<double>()) {
                    command["command"] = "left";
                } else {
                    command["command"] = "right";
                }

                double my_angle = my_car[1].get<double>();
                cerr << my_angle << endl;

                // normalize angle
                while (my_angle > PI) {
                    my_angle -= 2.0 * PI;
                }
                while (my_angle < -PI) {
                    my_angle += 2.0 * PI;
                }

                // roll over prevention (this feature can lead to death)
                if (my_angle > PI / 4.0) {
                    cerr << "Uhh!" << endl;
                    command["command"] = "left";
                } else if (my_angle < -PI / 4.0) {
                    cerr << "Ahh!" << endl;
                    command["command"] = "right";
                } else {
                    cerr << "Attack!" << endl;
                }
            }

            cout << command.dump() << endl;
            tick++;
        } else {
            break;
        }
    }

    return 0;
}