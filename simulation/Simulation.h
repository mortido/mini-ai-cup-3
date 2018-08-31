#ifndef MINI_AI_CUP_3_SIMULATOR_H
#define MINI_AI_CUP_3_SIMULATOR_H

#include <memory>

#include "Map.h"
#include "Car.h"
#include "TrashLine.h"
#include "../chipmunk/include/chipmunk.h"

#include "../../nlohmann/json.hpp"

using json = nlohmann::json;

class Simulation {
private:
    cpSpace *space;
    std::unique_ptr<Map> map;

public:
    int round;
    int tick_index;
    int my_player_id;
    int enemy_player_id;

    int my_lives;
    int enemy_lives;

    Simulation();
    virtual ~Simulation();

    void update_tick(const json &params);
    void new_round(const json &params);
    void simulate_tick();
};


#endif //MINI_AI_CUP_3_SIMULATOR_H
