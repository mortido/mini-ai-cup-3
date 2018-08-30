#include <stdio.h>
#include <iostream>
#include <chrono>

#include "chipmunk/include/chipmunk.h"
#include "../nlohmann/json.hpp"

#include "Randomizer.h"
#include "GameState.h"
#include "Solution.h"
#include "Solver.h"

using namespace std;
using namespace std::chrono;


int main() {
    string input_string;
    cin >> input_string;

    // read inputs
    auto state = nlohmann::json::parse(input_string);

    // init game constants
    GameConstants::initConstants(state);
    Randomizer::init();

    // make preparations
    Solver solver{};
    GameState game_state{};

    while (game_state.tick_index < GameConstants::MAX_GAME_TICKS()) {
        high_resolution_clock::time_point start_time(high_resolution_clock::now());

        // read inputs
        cin >> input_string;
        state = nlohmann::json::parse(input_string);
        game_state.update_from_json(state);

        // find best move
        solver.solve(game_state, start_time);

        // write best solution
        cout << solver.best_solutions[game_state.my_player_id].to_json().dump() << endl;
        cout << solver.best_solutions[game_state.my_player_id].fitness << endl;
        cout << solver.best_solutions[game_state.my_player_id].moves[0] << endl;
        game_state.next_tick();
    }

    return 0;
}