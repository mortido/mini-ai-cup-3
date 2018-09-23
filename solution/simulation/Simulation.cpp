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

    cars[0]->last_touched = -100;
    cars[1]->last_touched = -100;

    heapRestoreFrom(buffer, copy_size);
}

void Simulation::save() {
    saved_tick = sim_tick_index;
    copy_size = heapCopyTo(buffer);
}

static cpBool kill_car_on_button_press(cpArbiter *arb, cpSpace *space, bool *alive) {
    *alive = false;
//    return cpFalse;
    return cpTrue;
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

    map.reset(new Map(params["proto_map"], space, params["proto_car"]["external_id"].get<int>()));
    deadline.reset(new Deadline(Deadline::ASC, 1200, 800));

    cars[0].reset(new Car(params["proto_car"], space, 1.0, 0, GAME::LEFT_CAR_POS)); // 300-300
    cpCollisionHandler *ch1 = cpSpaceAddWildcardHandler(space, cars[0]->button_collision_type);
    ch1->preSolveFunc = (cpCollisionBeginFunc) kill_car_on_button_press;
    ch1->userData = &(cars[0]->alive);

    cars[1].reset(new Car(params["proto_car"], space, -1.0, 1, GAME::RIGHT_CAR_POS)); // 900-300
    cpCollisionHandler *ch2 = cpSpaceAddWildcardHandler(space, cars[1]->button_collision_type);
    ch2->preSolveFunc = (cpCollisionBeginFunc) kill_car_on_button_press;
    ch2->userData = &(cars[1]->alive);

    map->attach(space);
    deadline->attach(space);
    cars[0]->attach(space);
    cars[1]->attach(space);

    sim_tick_index = 0;
}

#include <chipmunk/chipmunk_structs.h>

static void
tocuhHelper(cpBody *body, cpArbiter *arb, bool *touched_by_enemy) {
    if (!*touched_by_enemy) {
        *touched_by_enemy = (arb->a->filter.group == 1 && arb->b->filter.group == 2) ||
                            (arb->a->filter.group == 2 && arb->b->filter.group == 1);
    }

}

void Simulation::step() {

//    bool touched_by_enemy{false};
//    cpBodyEachArbiter(cars[0]->car_body, (cpBodyArbiterIteratorFunc) tocuhHelper, &touched_by_enemy);
//    cpBodyEachArbiter(cars[0]->rear_wheel_body, (cpBodyArbiterIteratorFunc) tocuhHelper,
//                      &touched_by_enemy);
//    cpBodyEachArbiter(cars[0]->front_wheel_body, (cpBodyArbiterIteratorFunc) tocuhHelper,
//                      &touched_by_enemy);
//
//    if(touched_by_enemy){
//        cars[0]->last_touched=sim_tick_index;
//    }
//
//    touched_by_enemy =false;
//    cpBodyEachArbiter(cars[1]->car_body, (cpBodyArbiterIteratorFunc) tocuhHelper, &touched_by_enemy);
//    cpBodyEachArbiter(cars[1]->rear_wheel_body, (cpBodyArbiterIteratorFunc) tocuhHelper,
//                      &touched_by_enemy);
//    cpBodyEachArbiter(cars[1]->front_wheel_body, (cpBodyArbiterIteratorFunc) tocuhHelper,
//                      &touched_by_enemy);
//
//    if(touched_by_enemy){
//        cars[1]->last_touched=sim_tick_index;
//    }


    if (GAME::TICK_TO_DEADLINE - sim_tick_index < 1) {
        deadline->move();
//        if (sim_tick_index - saved_tick > GA::DEPTH / 2) {
//            deadline->move();
//        }
        if (sim_tick_index - saved_tick > GA::DEPTH * 0.75) {
            deadline->move();
        }
//        if (sim_tick_index - saved_tick > GA::DEPTH * 0.75) {
//            deadline->move();
//        }

    }
//    if (sim_tick_index - saved_tick > GA::DEPTH * 0.75) {
//        cpSpaceStep(space, GAME::SIMULATION_DT * 3);
//    }
//        else
    if (sim_tick_index - saved_tick > GA::DEPTH * 0.75) {
        sim_tick_index++;
        cpSpaceStep(space, GAME::SIMULATION_DT * 2);
    } else {
        cpSpaceStep(space, GAME::SIMULATION_DT);
    }
    sim_tick_index++;
}

#ifdef REWIND_VIEWER

void Simulation::draw(json &params, int my_player, std::array<Solution, 2> best_solutions) {
    rewind.message("FIT: %.6f\\n\\n", best_solutions[my_player].fitness);
    rewind.message("x_dif: %.6f\\n",
                   cpBodyGetPosition(cars[my_player]->car_body).x - params["my_car"][0][0].get<cpFloat>());
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

    for (int i = static_cast<int>(sim_tick_index > 0); i < GA::DEPTH; i++) {
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

cpFloat Simulation::get_closest_point_to_button(int player_id, bool ignore_cars) {
    cpVect p1 = cpBodyLocalToWorld(cars[player_id]->car_body, cpPolyShapeGetVert(cars[player_id]->button_shape, 0));
    cpVect p2 = cpBodyLocalToWorld(cars[player_id]->car_body, cpPolyShapeGetVert(cars[player_id]->button_shape, 1));
    cpVect cp_middle = cpvmult(p1 + p2, 0.5);

    auto f = cars[player_id]->car_filter;
    if (ignore_cars) {
        f = cpShapeFilterNew(cars[player_id]->car_group, cars[player_id]->car_category,
                             CP_ALL_CATEGORIES ^ cars[1 - player_id]->car_category);
    }
    cpPointQueryInfo queryInfo;
    double dist = 500.0;
    if (cpSpacePointQueryNearest(space, cp_middle, dist, f, &queryInfo)) {
        dist = std::min(dist, queryInfo.distance);
    }

    if (cpSpacePointQueryNearest(space, p1, dist, f, &queryInfo)) {
        dist = std::min(dist, queryInfo.distance);
    }

    if (cpSpacePointQueryNearest(space, p2, dist, f, &queryInfo)) {
        dist = std::min(dist, queryInfo.distance);
    }

//    if (cpSpacePointQueryNearest(space, cp_middle, dist,f, &queryInfo)) {
//        dist = std::min(dist, queryInfo.distance);
//    }
//
//    if (cpSpacePointQueryNearest(space, p1, dist, f, &queryInfo)) {
//        dist = std::min(dist, queryInfo.distance);
//    }
//
//    if (cpSpacePointQueryNearest(space, p2, dist, f, &queryInfo)) {
//        dist = std::min(dist, queryInfo.distance);
//    }

    return dist;
}

cpFloat Simulation::get_closest_point_to_button2(int player_id, bool ignore_cars) {
    cpVect p1 = cpBodyLocalToWorld(cars[player_id]->car_body, cpPolyShapeGetVert(cars[player_id]->button_shape, 0));
    cpVect p2 = cpBodyLocalToWorld(cars[player_id]->car_body, cpPolyShapeGetVert(cars[player_id]->button_shape, 1));
    cpVect cp_middle = cpvmult(p1 + p2, 0.5);

    auto f = cars[player_id]->car_filter;

    if (ignore_cars) {
        f = cpShapeFilterNew(cars[player_id]->car_group, cars[player_id]->car_category,
                             CP_ALL_CATEGORIES ^ cars[1 - player_id]->car_category);
    }
    cpSegmentQueryInfo queryInfo;
    double dist = 100.0;

    cpVect v = cpvnormalize(p2 - p1);

    cpVect normal = cpvperp(v);
    cpVect back = cpvrotate(v, cpvforangle(3.0 * PI / 4.0));
    cpVect front = cpvrotate(v, cpvforangle(PI / 4.0));

    if (cpSpaceSegmentQueryFirst(space, cp_middle, cp_middle + normal * dist, 1.0, f, &queryInfo)) {
        dist = std::min(dist, cpvdist(queryInfo.point, cp_middle));
    }

    if (cpSpaceSegmentQueryFirst(space, p1, p1 + back * dist, 1.0, f, &queryInfo)) {
        dist = std::min(dist, cpvdist(queryInfo.point, p1));
    }

    if (cpSpaceSegmentQueryFirst(space, p2, p2 + front * dist, 1.0, f, &queryInfo)) {
        dist = std::min(dist, cpvdist(queryInfo.point, p2));
    }

    return dist;
}


cpFloat Simulation::get_lowest_button_point(int player_id) {
    cpVect p1 = cpBodyLocalToWorld(cars[player_id]->car_body, cpPolyShapeGetVert(cars[player_id]->button_shape, 0));
    cpVect p2 = cpBodyLocalToWorld(cars[player_id]->car_body, cpPolyShapeGetVert(cars[player_id]->button_shape, 1));
    return std::min(std::min(cpvmult(p1 + p2, 0.5).y, p1.y), p2.y);
}

cpFloat Simulation::get_lowest_bus_point(int player_id) {

    for (int i=0;i<7;i++){

    }
    cpVect p1 = cpBodyLocalToWorld(cars[player_id]->car_body, cpPolyShapeGetVert(cars[player_id]->button_shape, 0));
    cpVect p2 = cpBodyLocalToWorld(cars[player_id]->car_body, cpPolyShapeGetVert(cars[player_id]->button_shape, 1));
    return std::min(std::min(cpvmult(p1 + p2, 0.5).y, p1.y), p2.y);
}

cpFloat Simulation::get_my_distance_to_enemy_button(int me, int enemy) {
    cpVect p1 = cpBodyLocalToWorld(cars[enemy]->car_body, cpPolyShapeGetVert(cars[enemy]->button_shape, 0));
    cpVect p2 = cpBodyLocalToWorld(cars[enemy]->car_body, cpPolyShapeGetVert(cars[enemy]->button_shape, 1));
    cpVect cp_middle = cpvmult(p1 + p2, 0.5);

    cpPointQueryInfo queryInfo;
//    if (cpSpacePointQueryNearest(space, cp_middle, 2000.0,
//                                 cpShapeFilterNew(cars[enemy]->car_group, cars[enemy]->car_category,
//                                                  cars[me]->car_category), &queryInfo)) {
//        return queryInfo.distance;
//    } else {
//        return 2000.0;
//    }
//
//
    double dist = 2000.0;
    auto f = cpShapeFilterNew(cars[enemy]->car_group, cars[enemy]->car_category, cars[me]->car_category);
    if (cpSpacePointQueryNearest(space, cp_middle, dist, f, &queryInfo)) {
        dist = std::min(dist, queryInfo.distance);
    }
    if (cpSpacePointQueryNearest(space, p1, dist, f, &queryInfo)) {
        dist = std::max(dist, queryInfo.distance);
    }
    if (cpSpacePointQueryNearest(space, p2, dist, f, &queryInfo)) {
        dist = std::max(dist, queryInfo.distance);
    }
//    dist = cpvdist(cpBodyGetPosition(cars[me]->rear_wheel_body), cp_middle) - cars[me]->rear_wheel_radius;
//    dist = std::min(dist, cpvdist(cpBodyGetPosition(cars[me]->front_wheel_body), cp_middle) - cars[me]->front_wheel_radius);
////
//    dist = std::min(dist, cpvdist(cpBodyGetPosition(cars[me]->rear_wheel_body), p1) - cars[me]->rear_wheel_radius);
//    dist = std::min(dist, cpvdist(cpBodyGetPosition(cars[me]->front_wheel_body), p1) - cars[me]->front_wheel_radius);
//
//    dist = std::min(dist, cpvdist(cpBodyGetPosition(cars[me]->rear_wheel_body), p2) - cars[me]->rear_wheel_radius);
//    dist = std::min(dist, cpvdist(cpBodyGetPosition(cars[me]->front_wheel_body), p2) - cars[me]->front_wheel_radius);
//
    return dist;
}

cpFloat Simulation::get_my_distance_to_enemy_button_2(int me, int enemy) {
    cpVect p1 = cpBodyLocalToWorld(cars[enemy]->car_body, cpPolyShapeGetVert(cars[enemy]->button_shape, 0));
    cpVect p2 = cpBodyLocalToWorld(cars[enemy]->car_body, cpPolyShapeGetVert(cars[enemy]->button_shape, 1));
    cpVect cp_middle = cpvmult(p1 + p2, 0.5);

    cpPointQueryInfo queryInfo;
//    if (cpSpacePointQueryNearest(space, cp_middle, 2000.0,
//                                 cpShapeFilterNew(cars[enemy]->car_group, cars[enemy]->car_category,
//                                                  cars[me]->car_category), &queryInfo)) {
//        return queryInfo.distance;
//    } else {
//        return 2000.0;
//    }
//
//
    double dist = 2000.0;
    auto f = cpShapeFilterNew(cars[enemy]->car_group, cars[enemy]->car_category, cars[me]->car_category);
    if (cpSpacePointQueryNearest(space, cp_middle, dist, f, &queryInfo)) {
        dist = std::min(dist, queryInfo.distance);
    }
    if (cpSpacePointQueryNearest(space, p1, dist, f, &queryInfo)) {
        dist = std::min(dist, queryInfo.distance);
    }
    if (cpSpacePointQueryNearest(space, p2, dist, f, &queryInfo)) {
        dist = std::min(dist, queryInfo.distance);
    }
//    dist = cpvdist(cpBodyGetPosition(cars[me]->rear_wheel_body), cp_middle) - cars[me]->rear_wheel_radius;
//    dist = std::min(dist, cpvdist(cpBodyGetPosition(cars[me]->front_wheel_body), cp_middle) - cars[me]->front_wheel_radius);
////
//    dist = std::min(dist, cpvdist(cpBodyGetPosition(cars[me]->rear_wheel_body), p1) - cars[me]->rear_wheel_radius);
//    dist = std::min(dist, cpvdist(cpBodyGetPosition(cars[me]->front_wheel_body), p1) - cars[me]->front_wheel_radius);
//
//    dist = std::min(dist, cpvdist(cpBodyGetPosition(cars[me]->rear_wheel_body), p2) - cars[me]->rear_wheel_radius);
//    dist = std::min(dist, cpvdist(cpBodyGetPosition(cars[me]->front_wheel_body), p2) - cars[me]->front_wheel_radius);
//
    return dist;
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

cpFloat Simulation::get_position_score(int player_id) {

    const cpVect &temp1 = cpBodyGetPosition(cars[player_id]->rear_wheel_body);
    const cpVect &temp2 = cpBodyGetPosition(cars[player_id]->front_wheel_body);
    return 0.5 * (map->weights[static_cast<int>(temp1.x / 10.0)][static_cast<int>(temp1.y / 10.0)] +
                  map->weights[static_cast<int>(temp2.x / 10.0)][static_cast<int>(temp2.y / 10.0)]);

//    const cpVect &p = cpBodyLocalToWorld(cars[player_id]->car_body,
//                                         cpBodyGetCenterOfGravity(cars[player_id]->car_body));
//    return map->weights[static_cast<int>(p.x / 10.0)][static_cast<int>(p.y / 10.0)];

}

double normilize_angle(double x) {
    x = fmod(x + PI, 2.0 * PI);
    if (x < 0.0)
        x += 2.0 * PI;
    return x - PI;
}

double Simulation::get_car_angle(int player_id) {
    return sim_tick_index ? normilize_angle(cpBodyGetAngle(cars[player_id]->car_body)) : 0.0;
}
