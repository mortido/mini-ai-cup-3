#include "GameState.h"

GameState::GameState() : tick_index{0} {

}

void GameState::next_tick() {
    tick_index++;
}

void GameState::update_from_json(const json &json) {
    my_player_id = 0;
    enemy_player_id = 1;
}
