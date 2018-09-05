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

static cpBool kill_car_on_button_press(cpArbiter *arb, cpSpace *space, bool *alive){
    *alive = false;
    return cpFalse;
}

void Simulation::new_round(const json &params) {
    tick_index = -1;
    round++;
    ticks_to_deadline = GAME::TICK_TO_DEADLINE;

    my_lives = params["my_lives"].get<int>();
    enemy_lives = params["enemy_lives"].get<int>();

    if (round == 0) {
        space = cpSpaceNew();
        cpSpaceSetGravity(space, GAME::GRAVITY); // 0 -700
        cpSpaceSetDamping(space, GAME::DAMPING); // 0.85
//        cpSpaceSetCollisionPersistence(space, 0);
    } else {
        map->detach(space);
        deadline->detach(space);
//        cars[0]->detach_shapes(space);
//        cars[1]->detach_shapes(space);
//        cars[0]->detach_bodies(space);
//        cars[1]->detach_bodies(space);
//        cars[0]->detach_constraints(space);
//        cars[1]->detach_constraints(space);
        cars[0]->detach(space);
        cars[1]->detach(space);
//        cpSpaceFree(space);
//        space = cpSpaceNew();
//        cpSpaceSetGravity(space, GAME::GRAVITY);
//        cpSpaceSetDamping(space, GAME::DAMPING);
    }

    map.reset(new Map(params["proto_map"], space));
    deadline.reset(new Deadline(Deadline::ASC, 1800, 800));

    cars[0].reset(new Car(params["proto_car"], space, 1.0, 0, GAME::LEFT_CAR_POS)); // 300-300
    cpCollisionHandler *ch1 = cpSpaceAddWildcardHandler(space, cars[0]->button_collision_type);
    ch1->beginFunc = (cpCollisionBeginFunc) kill_car_on_button_press;
    ch1->userData = &(cars[0]->alive);

    cars[1].reset(new Car(params["proto_car"], space, -1.0, 1, GAME::RIGHT_CAR_POS)); // 900-300
    cpCollisionHandler *ch2 = cpSpaceAddWildcardHandler(space, cars[1]->button_collision_type);
    ch2->beginFunc = (cpCollisionBeginFunc) kill_car_on_button_press;
    ch2->userData = &(cars[1]->alive);

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
//    deadline->update_from_json(params["deadline_position"].get<cpFloat>());
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
    deadline->reset();

    cpSpaceReindexShapesForBody(space, cars[0]->car_body);
    cpSpaceReindexShapesForBody(space, cars[0]->rear_wheel_body);
    cpSpaceReindexShapesForBody(space, cars[0]->front_wheel_body);
    cpSpaceReindexShapesForBody(space, cars[1]->car_body);
    cpSpaceReindexShapesForBody(space, cars[1]->rear_wheel_body);
    cpSpaceReindexShapesForBody(space, cars[1]->front_wheel_body);
    cpSpaceReindexStatic(space);
    cpSpaceReindexShape(space, deadline->shape);

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

cpFloat Simulation::get_closest_point_to_button(int player_id) {
//    cpShape *
//    cpSpacePointQueryNearest(cpSpace *space, cpVect point, cpFloat maxDistance, cpShapeFilter filter, cpPointQueryInfo *out)
//
//    cpPointQueryInfo queryInfo;
//
//    cpSpacePointQueryNearest(space, , 500.0, &queryInfo);
    return 0;
}
