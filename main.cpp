#include <stdio.h>
#include <iostream>
#include <chrono>

#include "chipmunk/include/chipmunk.h"
#include "../nlohmann/json.hpp"

#include "random/Randomizer.h"
#include "GameState.h"
#include "Solution.h"
#include "Solver.h"

using namespace std;
using namespace std::chrono;

#define NOW high_resolution_clock::now()
#define ELAPSED_TIME duration_cast<duration<double>>(NOW - start_time).count()

int main() {

    int round = 0;
    double time_bank = 120.0 - 10.0;

    // make preparations
    Solver solver{};
    GameState game_state{};
    Randomizer::init();

    high_resolution_clock::time_point start_time;
    string input_string, input_type;
    while (true) {
        getline(cin, input_string) ;
        start_time = NOW;

        // parse inputs
        auto state = nlohmann::json::parse(input_string);
        input_type = state["type"].get<std::string>();

        if (input_type == "new_match") {
            // init game constants
//            GameConstants::initConstants(state["params"]);
            game_state.reset(state["params"]);
            solver.set_time_limt(
                    time_bank / (game_state.my_lives + game_state.enemy_lives - 1) * GAME::MAX_ROUND_TICKS);
            round++;
        } else if (input_type == "tick") {
            game_state.update_from_json(state["params"]);

            // find best move
            solver.solve(game_state, start_time);

            // write best solution
            cout << solver.best_solutions[game_state.my_player_id].to_json().dump() << endl;
            cerr << "round " << round << " tick " << game_state.tick_index << endl;
            cerr << solver.best_solutions[game_state.my_player_id].fitness << endl;
            cerr << solver.best_solutions[game_state.my_player_id].moves[0] << endl;

#ifdef REWIND_VIEWER
            // TODO: simulate best_solutions again and draw them.
#endif

            game_state.next_tick();
        } else {
            // TODO: remove trash line because we are good
            break;
        }

        time_bank -= ELAPSED_TIME;
    }

    return 0;
}