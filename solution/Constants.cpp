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
    e = std::stod(argv[5]);
    f = std::stod(argv[6]);
    g = std::stod(argv[7]);
    h = std::stod(argv[8]);
    j = std::stod(argv[9]);
    i = std::stod(argv[10]);
}

GameConstants *GameConstants::INSTANCE() {
    return instance.get();
}

#endif