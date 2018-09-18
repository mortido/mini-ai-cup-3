#include <iostream>
#include <chrono>
#include <array>
#include <chipmunk/chipmunk.h>
#include "../../nlohmann/json.hpp"

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

template<typename... Args>
static inline std::string format(const char *fmt, Args... args) {
    static char buf[2048];
    int bytes = sprintf(buf, fmt, args...);
    buf[bytes] = '\0';
    return std::string(buf);
}

int main(int argc, char *argv[]) {
#ifdef OPTIMIZATION_RUN
    GameConstants::initConstants(argc, argv);
    Randomizer::init(stoi(argv[argc - 1]));
#else
    Randomizer::init();
#endif
    Simulation simulation;
    Solver solver;

    double time_bank = GAME::TIME_BANK;
    double time_limit = 0.020;
    int round{-1}, tick_index{0}, global_tick_index{0};

    int my_player_id{0}, enemy_player_id{1};
    int my_lives, enemy_lives;

    string input_string, input_type;
    high_resolution_clock::time_point start_time;

    array<enemyPrediction, GAME::MOVES_COUNT> enemyPredictions{};
    int my_prev_move{0}, my_prev_prev_move{0};

#ifdef LOCAL_RUN
    size_t max_bytes_to_copy = 0;
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
#ifdef CALC_TIME_BANK
            if (round == 0) {
                time_bank = time_bank * (my_lives + enemy_lives - 1) / 9.0;
                //TODO: output timebank
            }
#endif
            time_limit = time_bank / ((my_lives + enemy_lives - 1) * GAME::MAX_ROUND_TICKS);

#ifdef LOCAL_RUN
            if (round) {
                cerr << "round " << round << " pos_diff_sum: " << simulation.car_pos_error.x << " ; ";
                cerr << simulation.car_pos_error.y;
                cerr << " squared =" << state["params"]["proto_car"].value("squared_wheels", false) << endl;
                cerr << "BYTES TO COPY " << max_bytes_to_copy << endl;
                max_bytes_to_copy = 0;
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

            if (global_tick_index == 0) {
                // init player positions once per game
                solver.my_player_id = my_player_id = params["my_car"][2].get<int>() == -1;
                solver.enemy_player_id = enemy_player_id = 1 - my_player_id;
            }

            if (tick_index) {
                simulation.restore();
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

#ifdef LOCAL_RUN
            max_bytes_to_copy = std::max(max_bytes_to_copy, getBytesToCopy());
#endif
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

            my_prev_prev_move = my_prev_move;
            solver.solve(simulation, start_time, time_limit, my_prev_move);
            my_prev_move = solver.best_solutions[my_player_id].moves[tick_index > 0];
            auto command = solver.best_solutions[my_player_id].to_json(tick_index > 0);

            simulation.restore();
            if (tick_index) {
                simulation.step();
            }

            command["debug"] = format("TL: %.6f of %.6f\n", ELAPSED_TIME, time_limit) +
                               format("MY GENERATIONS(2x%d by %d): %.d\n", GA::POPULATION_SIZE, GA::DEPTH,
                                      solver.my_generations) +
                               format("MY SIMS: %d\n", solver.my_simulations) +
                               format("ENEMY GENERATIONS: %.d\n", solver.enemy_generations) +
                               format("ENEMY SIMS: %d\n", solver.enemy_simulations) +
                               format("x_dif: %.6f\n",
                                      cpBodyGetPosition(simulation.cars[my_player_id]->car_body).x -
                                      params["my_car"][0][0].get<cpFloat>()) +
                               format("y_dif: %.6f\n",
                                      cpBodyGetPosition(simulation.cars[my_player_id]->car_body).y -
                                      params["my_car"][0][1].get<cpFloat>()) +
                               format("rear_x_dif: %.6f\n",
                                      cpBodyGetPosition(simulation.cars[my_player_id]->rear_wheel_body).x -
                                      params["my_car"][3][0].get<cpFloat>()) +
                               format("rear_y_dif: %.6f\n",
                                      cpBodyGetPosition(simulation.cars[my_player_id]->rear_wheel_body).y -
                                      params["my_car"][3][1].get<cpFloat>()) +
                               format("front_x_dif: %.6f\n",
                                      cpBodyGetPosition(simulation.cars[my_player_id]->front_wheel_body).x -
                                      params["my_car"][4][0].get<cpFloat>()) +
                               format("front_y_dif: %.6f\n",
                                      cpBodyGetPosition(simulation.cars[my_player_id]->front_wheel_body).y -
                                      params["my_car"][4][1].get<cpFloat>());
            cout << command.dump() << endl;

            time_bank -= ELAPSED_TIME;

#ifdef LOCAL_RUN
            simulation.restore();
            if (tick_index) {
                simulation.move_car(my_player_id, my_prev_prev_move);
                simulation.move_car(enemy_player_id, solver.best_solutions[enemy_player_id].moves[0]);
                simulation.step();
            }
            simulation.check(my_player_id, params);
#ifdef REWIND_VIEWER
            simulation.rewind.message("TICK: %d ME: %d ENEMY: %d\\n", tick_index, my_lives, enemy_lives);
            simulation.rewind.message("TL: %.6f of %.6f\\n", ELAPSED_TIME, time_limit);
            simulation.rewind.message("MY GENERATIONS(2x%d by %d): %.d\\n", GA::POPULATION_SIZE, GA::DEPTH,
                                      solver.my_generations);
            simulation.rewind.message("MY SIMS: %d\\n", solver.my_simulations);
            simulation.rewind.message("ENEMY GENERATIONS: %.d\\n", solver.enemy_generations);
            simulation.rewind.message("ENEMY SIMS: %d\\n", solver.enemy_simulations);

            for (int j = 0; j < 10; j++) {
                simulation.rewind.message("COMP: %d %f\\n", j,
                                          solver.best_solutions[my_player_id].fitness_components[j]);
            }

            simulation.draw(params, my_player_id, solver.best_solutions);
#endif
#endif

            tick_index++;
            global_tick_index++;
        } else {
            break;
        }
    }

    return 0;
}
