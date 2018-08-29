#ifndef MINI_AI_CUP_3_CONSTANTS_H
#define MINI_AI_CUP_3_CONSTANTS_H

#include "../nlohmann/json.hpp"

using json = nlohmann::json;

namespace GA {
    constexpr int DEPTH = 100;
    constexpr int POPULATION_SIZE = 50;
    constexpr double MUTATION_PROBABILITY = 0.3;
    constexpr double TIME_LIMIT = 0.018;
//    constexpr int DEBUG_ITERATIONS_LIMIT = 10;
    constexpr int SOLVE_ENEMY_EVERY_N_TURNS = 3;
}

class GameConstants {

private:
    int max_game_ticks;

    static std::unique_ptr<GameConstants> instance;

public:
    static void initConstants(const json &json);

private:
    GameConstants(const json &json);

public:
    static int MAX_GAME_TICKS();
};

#endif //MINI_AI_CUP_3_CONSTANTS_H
