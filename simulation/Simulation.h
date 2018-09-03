#ifndef MINI_AI_CUP_3_SIMULATOR_H
#define MINI_AI_CUP_3_SIMULATOR_H

#include <memory>
#include <array>

#include "Map.h"
#include "Car.h"
#include "Deadline.h"
#include <chipmunk/chipmunk.h>

#include "../../nlohmann/json.hpp"

using json = nlohmann::json;

class Simulation {
public:
    int round;
    int tick_index;
    int sim_tick_index;
    int my_player_id;
    int enemy_player_id;
    int ticks_to_deadline;

    cpSpace *space;
    std::unique_ptr<Map> map;
    std::unique_ptr<Deadline> deadline;
    std::array<std::unique_ptr<Car>, 2> cars;

    int my_lives;
    int enemy_lives;

    Simulation();
    virtual ~Simulation();

    void update_tick(const json &params);
    void new_round(const json &params);
    void simulate_tick();
    void reset();

#ifdef REWIND_VIEWER
    void draw(json &params);
    RewindClient &rewind;
#endif
};

#endif //MINI_AI_CUP_3_SIMULATOR_H
