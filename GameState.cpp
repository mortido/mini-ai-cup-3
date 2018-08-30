#include "GameState.h"

GameState::GameState() : tick_index{0} {

}

void GameState::next_tick() {
    tick_index++;
}

void GameState::update_from_json(const json &params) {
    my_player_id = 0;
    enemy_player_id = 1;
}

void GameState::reset(const json &params) {
    my_player_id = -1;
    enemy_player_id = -1;

    my_lives = params["my_lives"].get<int>();
    enemy_lives = params["enemy_lives"].get<int>();
}
