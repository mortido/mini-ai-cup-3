#ifdef OPTIMIZATION_RUN

#include "Constants.h"
#include <memory>
#include <cstdlib>

std::unique_ptr<GameConstants> GameConstants::instance;

void GameConstants::initConstants(int argc, char *argv[]) {
    instance = std::unique_ptr<GameConstants>(new GameConstants(argc, argv));
}

GameConstants::GameConstants(int argc, char *argv[]) {
    mutate_type = std::stoi(argv[1]);
    crossover_type = std::stoi(argv[2]);
    randomization_type = std::stoi(argv[3]);
    aim_type = std::stoi(argv[4]);
    aim_theta = std::stoi(argv[5]);
    my_danger_theta = std::stoi(argv[6]);
    enemy_danger_theta = std::stoi(argv[7]);
    aim_safety_coeff = std::stod(argv[8]);
    aim_attack_coeff = std::stod(argv[9]);
    my_danger_coeff = std::stod(argv[10]);
    enemy_danger_coeff = std::stod(argv[11]);
    use_sigmoid = std::stoi(argv[12]);
    aim_safety_shift = std::stod(argv[13]);
    aim_attack_shift = std::stod(argv[14]);
    my_danger_shift = std::stod(argv[15]);
    enemy_danger_shift = std::stod(argv[16]);
    aim_safety_sig_coeff = std::stod(argv[17]);
    aim_attack_sig_coeff = std::stod(argv[18]);
    my_danger_sig_coeff = std::stod(argv[19]);
    enemy_danger_sig_coeff = std::stod(argv[20]);
}

GameConstants *GameConstants::INSTANCE() {
    return instance.get();
}

#endif