#include "Simulation.h"
#include "../Constants.h"

Simulation::Simulation() : space(cpSpaceNew()), round{-1} {
    cpSpaceSetGravity(space, cpv(GAME::X_GRAVITY, GAME::Y_GRAVITY));
}

Simulation::~Simulation() {
    cpSpaceFree(space);
}

void Simulation::new_round(const json &params) {
    tick_index = -1;
    round++;

    my_lives = params["my_lives"].get<int>();
    enemy_lives = params["enemy_lives"].get<int>();

    if (round == 0) {
        map = std::unique_ptr<Map>(new Map(params["proto_map"], space));
    } else {
        // implicitly call detach (in other case it will be called in destructor anyway)
        map->detach();
        map.reset(new Map(params["proto_map"], space));
    }
}

void Simulation::update_tick(const json &params) {
    tick_index++;

    // TODO: calc from json.
    my_player_id = 0;
    enemy_player_id = 1;

    // TODO: select enemy move from simulated positions.

    // TODO: update simulation according new speed/angle speed information.
}

void Simulation::simulate_tick() {
    cpSpaceStep(space, GAME::SIMULATION_DT);
}
