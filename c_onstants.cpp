#include "c_onstants.h"
#include <memory>

void GameConstants::initConstants(const json &json) {
    instance = std::unique_ptr<GameConstants>(new GameConstants(json));
}

GameConstants::GameConstants(const json &json) {
    this->max_game_ticks = json["GAME_TICKS"];
}

int GameConstants::MAX_GAME_TICKS() {
    return instance->max_game_ticks;
}