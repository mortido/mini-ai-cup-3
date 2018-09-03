#include "Simulation.h"
#include "../Constants.h"

Simulation::Simulation() : space(cpSpaceNew()),
#ifdef REWIND_VIEWER
                           rewind(RewindClient::instance()),
#endif
                           round{-1} {
    cpSpaceSetGravity(space, GAME::GRAVITY);
    cpSpaceSetDamping(space, GAME::DAMPING);
}

Simulation::~Simulation() {
    cpSpaceFree(space);
}

void Simulation::new_round(const json &params) {
    tick_index = -1;
    round++;
    ticks_to_deadline = GAME::TICK_TO_DEADLINE;

    my_lives = params["my_lives"].get<int>();
    enemy_lives = params["enemy_lives"].get<int>();

    if (round == 0) {
        map = std::unique_ptr<Map>(new Map(params["proto_map"], space));
        deadline = std::unique_ptr<Deadline>(new Deadline(Deadline::ASC, 1800, 800));
        cars[0] = std::unique_ptr<Car>(new Car(params["proto_car"], space, 1.0, 0, GAME::LEFT_CAR_POS));
        cars[1] = std::unique_ptr<Car>(new Car(params["proto_car"], space, -1.0, 1, GAME::RIGHT_CAR_POS));

    } else {
        map->detach(space);
        deadline->detach(space);
        cars[0]->detach(space);
        cars[1]->detach(space);

        cpSpaceFree(space);
        space = cpSpaceNew();
        cpSpaceSetGravity(space, GAME::GRAVITY);
        cpSpaceSetDamping(space, GAME::DAMPING);

        map.reset(new Map(params["proto_map"], space));
        deadline.reset(new Deadline(Deadline::ASC, 1800, 800));
        cars[0].reset(new Car(params["proto_car"], space, 1.0, 0, GAME::LEFT_CAR_POS));
        cars[1].reset(new Car(params["proto_car"], space, -1.0, 1, GAME::RIGHT_CAR_POS));
    }

    cpCollisionHandler *ch1 = cpSpaceAddWildcardHandler(space, 10);
    cpCollisionHandler *ch2 = cpSpaceAddWildcardHandler(space, 20);

    map->attach(space);
    deadline->attach(space);
    cars[0]->attach(space);
    cars[1]->attach(space);
}

//#include <iostream>

void Simulation::update_tick(const json &params) {
    tick_index++;
    sim_tick_index = tick_index;
//    std::cerr << tick_index << ":";
    if (tick_index == 0) {
        if (params["my_car"][2].get<int>() == 1) {
            my_player_id = 0;
            enemy_player_id = 1;
        } else {
            my_player_id = 1;
            enemy_player_id = 0;
        }
        cars[my_player_id]->set_from_json(params["my_car"]);
        cars[enemy_player_id]->set_from_json(params["enemy_car"]);
    } else {
//        cars[my_player_id]->update_from_json(params["my_car"]);
//        cars[enemy_player_id]->update_from_json(params["enemy_car"]);
    }
}

void Simulation::simulate_tick() {
    if (ticks_to_deadline - sim_tick_index < 1) {
        deadline->move();
    }

    cpSpaceStep(space, GAME::SIMULATION_DT);
    sim_tick_index++;
}

#ifdef REWIND_VIEWER

void Simulation::draw() {
    map->draw(rewind);
    cars[0]->draw(rewind);
    cars[1]->draw(rewind);
    rewind.end_frame();
}

#endif

void Simulation::reset() {
    sim_tick_index = tick_index;
    cars[0]->reset();
    cars[1]->reset();

    // TODO: reset deadline.
}
