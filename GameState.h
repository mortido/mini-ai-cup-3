#ifndef MINI_AI_CUP_3_GAMESTATE_H
#define MINI_AI_CUP_3_GAMESTATE_H

#include "../nlohmann/json.hpp"

using json = nlohmann::json;

class GameState {
public:
    int tick_index;
    int my_player_id;
    int enemy_player_id;

    int my_lives;
    int enemy_lives;

    GameState();
    void update_from_json(const json &json);
    void reset(const json &json);
    void next_tick();
};


#endif //MINI_AI_CUP_3_GAMESTATE_H
