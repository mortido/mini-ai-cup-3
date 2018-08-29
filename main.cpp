#include <stdio.h>
#include <iostream>
#include <chrono>

#include "chipmunk/include/chipmunk.h"
#include "../nlohmann/json.hpp"

#include "solution.h"

using namespace std;


#define NOW chrono::high_resolution_clock::now()
#define ELAPSED_TIME chrono::duration_cast<chrono::duration<double>>(NOW - start_time).count()


// ***********************************************************

int main() {

    chrono::high_resolution_clock::time_point start_time(NOW);

    string input_string;
    cin >> input_string;

    // read inputs
    auto state = nlohmann::json::parse(input_string);

    // parse state, init scene, constants, etc.

    Solution *best = nullptr;

    while (true) {
        start_time = NOW;

        // read inputs
        state = nlohmann::json::parse(input_string);
        // parse, update state




        double time_limit{0.018};

        while (ELAPSED_TIME < time_limit) {

        }

        if (rand()) {
            break;
        }

        // write best solution
        cout << best->to_json().dump() << endl;
    }

    return 0;
}