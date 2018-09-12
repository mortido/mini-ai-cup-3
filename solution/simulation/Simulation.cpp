#include "Simulation.h"
#include "../Constants.h"

Simulation::Simulation() : space{nullptr}, sim_tick_index{0}
#ifdef REWIND_VIEWER
        , rewind(RewindClient::instance())
#endif
{
    // Allocate heap for space and buffer to copy state.
    heap = malloc(HEAP_SIZE);
    buffer = malloc(HEAP_SIZE);
    heapInit((unsigned char *) heap, HEAP_SIZE);
}

Simulation::~Simulation() {
    cpSpaceFree(space);
    free(heap);
    free(buffer);
}

void Simulation::restore() {
    cars[0]->alive = true;
    cars[1]->alive = true;
    sim_tick_index = saved_tick;
    heapRestoreFrom(buffer, copy_size);
}

void Simulation::save() {
    saved_tick = sim_tick_index;
    copy_size = heapCopyTo(buffer);
}

static cpBool kill_car_on_button_press(cpArbiter *arb, cpSpace *space, bool *alive) {

    *alive = false;
    return cpFalse;
}

void Simulation::new_round(const json &params) {
#ifdef LOCAL_RUN
    car_pos_error.x = 0.0;
    car_pos_error.y = 0.0;
#endif

    if (space) {
        map->detach(space);
        deadline->detach(space);
        cars[0]->detach(space);
        cars[1]->detach(space);
    } else {
        space = cpSpaceNew();
        cpSpaceSetGravity(space, GAME::GRAVITY); // 0 -700
        cpSpaceSetDamping(space, GAME::DAMPING); // 0.85
    }

    // That will clear memory before object creation.
    map.reset(nullptr);
    deadline.reset(nullptr);
    cars[0].reset(nullptr);
    cars[1].reset(nullptr);

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

    sim_tick_index = 0;
}

void Simulation::step() {
    if (GAME::TICK_TO_DEADLINE - sim_tick_index < 1) {
        deadline->move();
    }

    cpSpaceStep(space, GAME::SIMULATION_DT);
    sim_tick_index++;
}

#ifdef REWIND_VIEWER

void Simulation::draw(json &params, int my_player, std::array<Solution, 2> best_solutions) {
    rewind.message("FIT: %.6f\\n\\n", best_solutions[my_player].fitness);
    rewind.message("x_dif: %.6f\\n",                   cpBodyGetPosition(cars[my_player]->car_body).x - params["my_car"][0][0].get<cpFloat>());
    rewind.message("y_dif: %.6f\\n",
                   cpBodyGetPosition(cars[my_player]->car_body).y - params["my_car"][0][1].get<cpFloat>());
    rewind.message("rear_x_dif: %.6f\\n",
                   cpBodyGetPosition(cars[my_player]->rear_wheel_body).x - params["my_car"][3][0].get<cpFloat>());
    rewind.message("rear_y_dif: %.6f\\n",
                   cpBodyGetPosition(cars[my_player]->rear_wheel_body).y - params["my_car"][3][1].get<cpFloat>());
    rewind.message("front_x_dif: %.6f\\n",
                   cpBodyGetPosition(cars[my_player]->front_wheel_body).x - params["my_car"][4][0].get<cpFloat>());
    rewind.message("front_y_dif: %.6f\\n",
                   cpBodyGetPosition(cars[my_player]->front_wheel_body).y - params["my_car"][4][1].get<cpFloat>());

    rewind.message("deadline_dif: %.6f\\n",
                   cpBodyGetPosition(deadline->body).y - params["deadline_position"].get<cpFloat>());

    rewind.message("\\nleft car:\\n");
    for (int i = 0; i < GA::DEPTH; i++) {
        rewind.message("%d, ", best_solutions[0].moves[i]);
    }
    rewind.message("\\nright car:\\n");
    for (int i = 0; i < GA::DEPTH; i++) {
        rewind.message("%d, ", best_solutions[1].moves[i]);
    }
    rewind.message("\\n");

    map->draw(rewind);

    rewind.circle(params["my_car"][3][0].get<double>(), params["my_car"][3][1].get<double>(),
                  cars[0]->rear_wheel_radius, 0x3FCC0000);
    rewind.circle(params["my_car"][4][0].get<double>(), params["my_car"][4][1].get<double>(),
                  cars[0]->front_wheel_radius, 0x3FCC0000);
//
    cars[0]->draw(rewind);
    cars[1]->draw(rewind);
    deadline->draw(rewind);

    rewind.CurrentLayer = 2;

    for (int i = 0; i < GA::DEPTH; i++) {
        cars[0]->move(best_solutions[0].moves[i]);
        cars[1]->move(best_solutions[1].moves[i]);
        step();
        cars[0]->draw(rewind, true);
        cars[1]->draw(rewind, true);
    }

    rewind.CurrentLayer = rewind.DEFAULT_LAYER;


    rewind.end_frame();
}

#endif

cpFloat Simulation::get_closest_point_to_button(int player_id) {
    cpVect p1 = cpBodyLocalToWorld(cars[player_id]->car_body, cpPolyShapeGetVert(cars[player_id]->button_shape, 0));
    cpVect p2 = cpBodyLocalToWorld(cars[player_id]->car_body, cpPolyShapeGetVert(cars[player_id]->button_shape, 1));
    cpVect cp_middle = cpvmult(p1+p2,0.5);

    cpPointQueryInfo queryInfo;
    if(cpSpacePointQueryNearest(space, cp_middle, 500.0, cars[player_id]->car_filter, &queryInfo)){
        return queryInfo.distance;
    }else{
        return 500.0;
    }
}

cpFloat Simulation::get_button_lowest_position(int player_id) {
    cpVect p1 = cpBodyLocalToWorld(cars[player_id]->car_body, cpPolyShapeGetVert(cars[player_id]->button_shape, 0));
    cpVect p2 = cpBodyLocalToWorld(cars[player_id]->car_body, cpPolyShapeGetVert(cars[player_id]->button_shape, 1));
    return std::min(std::min(cpvmult(p1+p2,0.5).y, p1.y), p2.y);
}

cpFloat Simulation::get_my_distance_to_enemy_button(int me, int enemy) {
    cpVect p1 = cpBodyLocalToWorld(cars[enemy]->car_body, cpPolyShapeGetVert(cars[enemy]->button_shape, 0));
    cpVect p2 = cpBodyLocalToWorld(cars[enemy]->car_body, cpPolyShapeGetVert(cars[enemy]->button_shape, 1));
    cpVect cp_middle = cpvmult(p1+p2,0.5);

    cpPointQueryInfo queryInfo;
    if(cpSpacePointQueryNearest(space, cp_middle, 2000.0,
            cpShapeFilterNew(cars[enemy]->car_group, cars[enemy]->car_category,cars[me]->car_category), &queryInfo)){
        return queryInfo.distance;
    }else{
        return 2000.0;
    }
}

#ifdef LOCAL_RUN

void Simulation::check(int my_player_id, const json &params) {
    car_pos_error.x += abs(cpBodyGetPosition(cars[my_player_id]->car_body).x - params["my_car"][0][0].get<cpFloat>());
    car_pos_error.y += abs(cpBodyGetPosition(cars[my_player_id]->car_body).y - params["my_car"][0][1].get<cpFloat>());
}

#endif

void Simulation::move_car(int player_id, int move) {
    cars[player_id]->move(move);
}
