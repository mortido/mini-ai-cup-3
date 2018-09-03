#include "Simulation.h"
#include "../Constants.h"

Simulation::Simulation() :
#ifdef REWIND_VIEWER
                           rewind(RewindClient::instance()),
#endif
                           round{-1} {

}

Simulation::~Simulation() {
    cpSpaceFree(space);
}

static void RemoveEachBody(cpBody *body, cpSpace *spc){
    cpSpaceRemoveBody(spc, body);
}

static void RemoveEachShape(cpShape *shape, cpSpace *spc){
    cpSpaceRemoveShape(spc, shape);
}

static void RemoveEachConstraint(cpConstraint *constraint, cpSpace *spc){
    cpSpaceRemoveConstraint(spc, constraint);
}

void Simulation::new_round(const json &params) {
    tick_index = -1;
    round++;
    ticks_to_deadline = GAME::TICK_TO_DEADLINE;

    my_lives = params["my_lives"].get<int>();
    enemy_lives = params["enemy_lives"].get<int>();

    if (round == 0) {
        space = cpSpaceNew();
        cpSpaceSetGravity(space, GAME::GRAVITY);
        cpSpaceSetDamping(space, GAME::DAMPING);
//        cpSpaceSetCollisionPersistence(space, 1);

        map = std::unique_ptr<Map>(new Map(params["proto_map"], space));
        deadline = std::unique_ptr<Deadline>(new Deadline(Deadline::ASC, 1800, 800));
        cars[0] = std::unique_ptr<Car>(new Car(params["proto_car"], space, 1.0, 0, GAME::LEFT_CAR_POS));
        cars[1] = std::unique_ptr<Car>(new Car(params["proto_car"], space, -1.0, 1, GAME::RIGHT_CAR_POS));

    } else {

//        cpSpaceEachShape(space, (cpSpaceShapeIteratorFunc)RemoveEachShape, space);
//        cpSpaceEachBody(space, (cpSpaceBodyIteratorFunc)RemoveEachBody, space);
//        cpSpaceEachConstraint(space, (cpSpaceConstraintIteratorFunc)RemoveEachConstraint, space);

        map->detach(space);
        deadline->detach(space);
        cars[0]->detach(space);
        cars[1]->detach(space);

//        cpSpaceFree(space);
//        space = cpSpaceNew();
//        cpSpaceSetGravity(space, GAME::GRAVITY);
//        cpSpaceSetDamping(space, GAME::DAMPING);
//        cpSpaceSetCollisionPersistence(space, 1);

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
//        cars[my_player_id]->set_from_json(params["my_car"]);
//        cars[enemy_player_id]->set_from_json(params["enemy_car"]);
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

void Simulation::draw(json &params) {
    map->draw(rewind);

    rewind.circle(params["my_car"][3][0].get<double>(), params["my_car"][3][1].get<double>(), cars[0]->rear_wheel_radius, 0x3FCC0000);
    rewind.circle(params["my_car"][4][0].get<double>(), params["my_car"][4][1].get<double>(), cars[0]->front_wheel_radius, 0x3FCC0000);

    cars[0]->draw(rewind);
    cars[1]->draw(rewind);
    rewind.end_frame();
}

#endif

void Simulation::reset() {
    sim_tick_index = tick_index;
    cars[0]->reset();
    cars[1]->reset();

//    map->detach(space);
//    deadline->detach(space);
//    cars[0]->detach(space);
//    cars[1]->detach(space);
//
//    cpSpaceFree(space);
//    space = cpSpaceNew();
//    cpSpaceSetGravity(space, GAME::GRAVITY);
//    cpSpaceSetDamping(space, GAME::DAMPING);

//    cpCollisionHandler *ch1 = cpSpaceAddWildcardHandler(space, 10);
//    cpCollisionHandler *ch2 = cpSpaceAddWildcardHandler(space, 20);
//
//    map->attach(space);
//    deadline->attach(space);
//    cars[0]->attach(space);
//    cars[1]->attach(space);

    // TODO: reset deadline.
}
