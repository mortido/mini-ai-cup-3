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
    constexpr double TIME_LIMIT = 0.018;
    constexpr int DEBUG_ITERATIONS_LIMIT = 100;
    constexpr int SOLVE_ENEMY_EVERY_N_TURNS = 3;
}

namespace GAME {
    constexpr int MOVES_COUNT = 3;
}

class GameConstants {

private:
    int max_game_ticks;

    static std::unique_ptr<GameConstants> instance;

    explicit GameConstants(const json &json);

public:
    static void initConstants(const json &json);

    static inline int MAX_GAME_TICKS();
};

#endif //MINI_AI_CUP_3_CONSTANTS_H
