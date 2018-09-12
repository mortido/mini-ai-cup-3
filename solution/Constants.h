#ifndef MINI_AI_CUP_3_CONSTANTS_H
#define MINI_AI_CUP_3_CONSTANTS_H

#include "../../nlohmann/json.hpp"
#include <chipmunk/chipmunk.h>

using json = nlohmann::json;

namespace GA {
    constexpr int DEPTH = 42;
    constexpr int POPULATION_SIZE = 5; // min 4;
    constexpr double THETA = (1.0 - 1.0 / GA::DEPTH);
    constexpr double THETA_PLUS = (1.0 + 1.0 / GA::DEPTH);

    constexpr double MUTATION_PROBABILITY = 0.3;
    constexpr double STOP_SLICE_MUTATE_PROBABILITY = 0.5;
    constexpr double CHANGE_MOVE_ON_RANDOMIZATION_PROBABILITY = 0.05;

    constexpr int DEBUG_ITERATIONS_LIMIT = 100;
    constexpr double ENEMY_TIME_COEFF = 0.33;
}

namespace GAME {
    constexpr double TIME_BANK = 120.0;

    constexpr int MOVES_COUNT = 3;
    constexpr int MAX_ROUND_TICKS = 1000;

    constexpr double SIMULATION_DT = 0.016;
    constexpr double SIMULATION_DT_INVERSED = 62.5;

    const cpVect GRAVITY = cpv(0.0, -700.0);
    constexpr double DAMPING = 0.85;
    constexpr int TICK_TO_DEADLINE = 600;
    const cpVect LEFT_CAR_POS = cpv(300.0, 300.0);
    const cpVect RIGHT_CAR_POS = cpv(900.0, 300.0);
    constexpr double MAX_WIDTH = 1200.0;
    constexpr double MAX_HEIGHT = 800.0;

    constexpr double MAP_FRICTION = 1.0;
    constexpr double MAP_ELASTICITY = 0.0;
}

#ifdef OPTIMIZATION_RUN

class GameConstants {

private:

    static std::unique_ptr<GameConstants> instance;
    GameConstants(int argc, char *argv[]);

public:
    int  mutate_type;
    int crossover_type;
    int randomization_type;

    int aim_type;
    int aim_theta;
    int my_danger_theta;
    int enemy_danger_theta;

    double aim_safety_coeff;
    double aim_attack_coeff;
    double my_danger_coeff;
    double enemy_danger_coeff;

    int use_sigmoid;
    double aim_safety_shift;
    double aim_attack_shift;
    double my_danger_shift;
    double enemy_danger_shift;

    double aim_safety_sig_coeff;
    double aim_attack_sig_coeff;
    double my_danger_sig_coeff;
    double enemy_danger_sig_coeff;


    static void initConstants(int argc, char *argv[]);

    static GameConstants* INSTANCE();
};
#endif

#endif //MINI_AI_CUP_3_CONSTANTS_H




