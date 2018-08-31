#ifndef MINI_AI_CUP_3_CONSTANTS_H
#define MINI_AI_CUP_3_CONSTANTS_H

#include "../nlohmann/json.hpp"

using json = nlohmann::json;

namespace GA {
    constexpr int DEPTH = 100;
    constexpr int POPULATION_SIZE = 50; // min 4;
    constexpr double MUTATION_PROBABILITY = 0.3;
    constexpr double STOP_SLICE_MUTATE_PROBABILITY = 0.5;
    constexpr double CHANGE_MOVE_ON_RANDOMIZATION_PROBABILITY = 0.05;
    constexpr int DEBUG_ITERATIONS_LIMIT = 100;
    constexpr int SOLVE_ENEMY_EVERY_N_TURNS = 3; // 1 enemy calculation, 2 my...
}

namespace GAME {
//    constexpr int MOVES_COUNT = 3;
    constexpr int MAX_ROUND_TICKS = 1000;

    constexpr double SIMULATION_DT = 0.016;

    constexpr double X_GRAVITY = 0.0;
    constexpr double Y_GRAVITY = -700.0;
    constexpr double MAX_WIDTH = 1200.0;
    constexpr double MAX_HEIGHT = 800.0;

    constexpr double MAP_FRICTION = 1.0;
    constexpr double MAP_ELASTICITY = 0.0;
}

//class GameConstants {
//
//private:
//    int max_game_ticks;
//
//    static std::unique_ptr<GameConstants> instance;
//    GameConstants(const json &json);
//
//public:
//    static void initConstants(const json &json);
//
//    static int MAX_GAME_TICKS();
//};

#endif //MINI_AI_CUP_3_CONSTANTS_H
