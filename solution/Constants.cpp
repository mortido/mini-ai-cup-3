#ifdef OPTIMIZATION_RUN

#include "Constants.h"
#include <memory>
#include <cstdlib>

std::unique_ptr<GameConstants> GameConstants::instance;

void GameConstants::initConstants(int argc, char *argv[]) {
    instance = std::unique_ptr<GameConstants>(new GameConstants(argc, argv));
}

GameConstants::GameConstants(int argc, char *argv[]) {
    a = std::stod(argv[1]);
    b = std::stod(argv[2]);
    c = std::stod(argv[3]);
    d = std::stod(argv[4]);
    h = std::stod(argv[5]);
    i = std::stod(argv[6]);
    e = std::stod(argv[7]);
    f = std::stod(argv[8]);
    g = std::stoi(argv[9]);
}

GameConstants *GameConstants::INSTANCE() {
    return instance.get();
}

#endif