#include <stdio.h>
#include <iostream>
#include <chrono>

#include "chipmunk/include/chipmunk.h"
#include "../nlohmann/json.hpp"

#include "s_olution.h"
#include "s_olver.h"

using namespace std;
using namespace std::chrono;


int main() {
    string input_string;
    cin >> input_string;

    // read inputs
    auto state = nlohmann::json::parse(input_string);

    // init game constants
    GameConstants::initConstants(state);

    // make preparations
    Solver solver{};

    while (0 < GameConstants::MAX_GAME_TICKS()) {
        high_resolution_clock::time_point start_time(high_resolution_clock::now());

        // read inputs
        state = nlohmann::json::parse(input_string);
        // TODO: parse, update state

        // find best move
        solver.solve(start_time);

        // write best solution
        cout << solver.my_best.to_json().dump() << endl;
    }

    return 0;
}