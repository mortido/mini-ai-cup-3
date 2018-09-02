#include "Simulation.h"
#include "../Constants.h"

Simulation::Simulation() : space(cpSpaceNew()),
#ifdef REWIND_VIEWER
                           rewind(RewindClient::instance()),
#endif
                           round{-1} {
    cpSpaceSetGravity(space, GAME::GRAVITY);
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

        // create left and right cars
        cars[0] = std::unique_ptr<Car>(new Car(params["proto_car"], space, 1.0, 0));
        cars[1] = std::unique_ptr<Car>(new Car(params["proto_car"], space, -1.0, 1));
    } else {
        // implicitly call detach (in other case it will be called in destructor anyway)
        map->detach();
        cars[0]->detach();
        cars[1]->detach();

        map.reset(new Map(params["proto_map"], space));
        cars[0].reset(new Car(params["proto_car"], space, 1.0, 0));
        cars[1].reset(new Car(params["proto_car"], space, 1.0, 1));
    }
}

void Simulation::update_tick(const json &params) {
    tick_index++;

    if (tick_index == 0) {
        if (params["my_car"][2].get<int>() == 1) {
            my_player_id = 0;
            enemy_player_id = 1;
        } else {
            my_player_id = 1;
            enemy_player_id = 0;
        }
        cars[my_player_id].set(params["my_car"]);
        cars[enemy_player_id].set(params["enemy_car"]);
    } else {
        cars[my_player_id].update(params["my_car"]);
        cars[enemy_player_id].update(params["enemy_car"]);
    }
}

void Simulation::simulate_tick() {
    cpSpaceStep(space, GAME::SIMULATION_DT);
}

#ifdef REWIND_VIEWER
void Simulation::draw() {
    map->draw(rewind);
    cars[0]->draw(rewind);
    cars[1]->draw(rewind);
}
#endif
