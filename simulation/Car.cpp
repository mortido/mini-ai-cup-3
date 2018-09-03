#include "Car.h"
#include "../Constants.h"
#include <vector>
#include <array>

Car::Car(const json &params, cpSpace *_space, double mirror,
         int _player_id, cpVect pos) : space_attached{_space}, player_id{_player_id},
                           car_speed{cpvzero}, rear_wheel_speed{cpvzero}, front_wheel_speed{cpvzero},
                           car_angle_speed{0.0}, rear_wheel_angle_speed{0.0}, front_wheel_angle_speed{0.0} {
    car_group = static_cast<cpGroup>(player_id + 1);
    button_collision_type = static_cast<cpCollisionType>((player_id + 1) * 10);

    torque = params["torque"].get<cpFloat>();
    max_speed = params["max_speed"].get<cpFloat>();
//    external_id = params["external_id"].get<int>();
    drive_type = params["drive"].get<DRIVE::Type>();
    squared_wheels = params.value("squared_wheels", false);

    auto car_body_mass = params["car_body_mass"].get<cpFloat>();

    std::vector<cpVect> car_poly;
    for (auto v: params["car_body_poly"]) {
        car_poly.emplace_back(cpv(v[0].get<cpFloat>() * mirror, v[1].get<cpFloat>()));
    }

    car_body = cpBodyNew(car_body_mass,
                         cpMomentForPoly(car_body_mass, static_cast<int>(car_poly.size()), car_poly.data(), cpvzero,
                                         0.0));
    car_shape = cpPolyShapeNew(car_body, static_cast<int>(car_poly.size()), car_poly.data(), cpTransformIdentity, 0.0);
    cpShapeSetFriction(car_shape, params["car_body_friction"].get<cpFloat>());
    cpShapeSetElasticity(car_shape, params["car_body_elasticity"].get<cpFloat>());
    car_filter = cpShapeFilterNew(car_group, CP_ALL_CATEGORIES, CP_ALL_CATEGORIES);
    cpShapeSetFilter(car_shape, car_filter);

    std::vector<cpVect> button_poly;
    for (auto v: params["button_poly"]) {
        button_poly.emplace_back(cpv(v[0].get<cpFloat>() * mirror, v[1].get<cpFloat>()));
    }
    button_shape = cpPolyShapeNew(car_body, static_cast<int>(button_poly.size()), button_poly.data(),
                                  cpTransformIdentity, 0.0);
    cpShapeSetFilter(button_shape, car_filter);
    cpShapeSetSensor(button_shape, static_cast<cpBool>(true));
    cpShapeSetCollisionType(button_shape, button_collision_type);

    cpBodySetCenterOfGravity(car_body, cpShapeGetCenterOfGravity(car_shape));

    // rear wheel ******************************************************************************************

    auto rear_wheel_mass = params["rear_wheel_mass"].get<cpFloat>();
    rear_wheel_radius = params["rear_wheel_radius"].get<cpFloat>();

    if (squared_wheels) {
        cpFloat dr = rear_wheel_radius * 2.0;
        rear_wheel_body = cpBodyNew(rear_wheel_mass, cpMomentForBox(rear_wheel_mass, dr, dr));
        cpBodySetPosition(rear_wheel_body, cpv(params["rear_wheel_position"][0].get<cpFloat>() * mirror,
                                               params["rear_wheel_position"][1].get<cpFloat>()));
        rear_wheel_shape = cpBoxShapeNew(rear_wheel_body, dr, dr, 0.0);
    } else {
        rear_wheel_body = cpBodyNew(rear_wheel_mass,
                                    cpMomentForCircle(rear_wheel_mass, 0.0, rear_wheel_radius, cpvzero));
        cpBodySetPosition(rear_wheel_body, cpv(params["rear_wheel_position"][0].get<cpFloat>() * mirror,
                                               params["rear_wheel_position"][1].get<cpFloat>()));
        rear_wheel_shape = cpCircleShapeNew(rear_wheel_body, rear_wheel_radius, cpvzero);
    }

    cpShapeSetFilter(rear_wheel_shape, car_filter);
    cpShapeSetFriction(rear_wheel_shape, params["rear_wheel_friction"].get<cpFloat>());
    cpShapeSetElasticity(rear_wheel_shape, params["rear_wheel_elasticity"].get<cpFloat>());

    rear_wheel_joint = cpPinJointNew(rear_wheel_body, car_body, cpvzero,
                                     cpv(params["rear_wheel_joint"][0].get<cpFloat>() * mirror,
                                         params["rear_wheel_joint"][1].get<cpFloat>()));

    rear_wheel_dump = cpDampedSpringNew(rear_wheel_body,
                                        car_body,
                                        cpvzero, cpv(params["rear_wheel_damp_position"][0].get<cpFloat>() * mirror,
                                                     params["rear_wheel_damp_position"][1].get<cpFloat>()),
                                        params["rear_wheel_damp_length"].get<cpFloat>(),
                                        params["rear_wheel_damp_stiffness"].get<cpFloat>(),
                                        params["rear_wheel_damp_damping"].get<cpFloat>());

    std::array<cpVect, 4> stop_points{cpv(0.0, 0.0),
                                      cpv(0.0, 1.0),
                                      cpv(rear_wheel_radius * 2.0 * mirror, 1),
                                      cpv(rear_wheel_radius * 2.0 * mirror, 0)};

    rear_wheel_stop = cpPolyShapeNew(car_body,
                                     4,
                                     stop_points.data(),
                                     cpTransformNew(1, 0, 0, 1,
                                                    params["rear_wheel_damp_position"][0].get<cpFloat>() *
                                                    mirror - rear_wheel_radius * mirror,
                                                    params["rear_wheel_damp_position"][1].get<cpFloat>()),
                                     0.0);

    // front wheel ******************************************************************************************
    auto front_wheel_mass = params["front_wheel_mass"].get<cpFloat>();
    front_wheel_radius = params["front_wheel_radius"].get<cpFloat>();

    if (squared_wheels) {
        cpFloat dr = front_wheel_radius * 2.0;
        front_wheel_body = cpBodyNew(front_wheel_mass, cpMomentForBox(front_wheel_mass, dr, dr));
        cpBodySetPosition(front_wheel_body, cpv(params["front_wheel_position"][0].get<cpFloat>() * mirror,
                                                params["front_wheel_position"][1].get<cpFloat>()));
        front_wheel_shape = cpBoxShapeNew(front_wheel_body, dr, dr, 0.0);
    } else {
        front_wheel_body = cpBodyNew(front_wheel_mass,
                                     cpMomentForCircle(front_wheel_mass, 0.0, front_wheel_radius, cpvzero));
        cpBodySetPosition(front_wheel_body, cpv(params["front_wheel_position"][0].get<cpFloat>() * mirror,
                                                params["front_wheel_position"][1].get<cpFloat>()));
        front_wheel_shape = cpCircleShapeNew(front_wheel_body, front_wheel_radius, cpvzero);
    }

    cpShapeSetFilter(front_wheel_shape, car_filter);
    cpShapeSetFriction(front_wheel_shape, params["front_wheel_friction"].get<cpFloat>());
    cpShapeSetElasticity(front_wheel_shape, params["front_wheel_elasticity"].get<cpFloat>());

    front_wheel_joint = cpPinJointNew(front_wheel_body, car_body, cpvzero,
                                      cpv(params["front_wheel_joint"][0].get<cpFloat>() * mirror,
                                          params["front_wheel_joint"][1].get<cpFloat>()));

    front_wheel_dump = cpDampedSpringNew(front_wheel_body,
                                         car_body,
                                         cpvzero, cpv(params["front_wheel_damp_position"][0].get<cpFloat>() * mirror,
                                                      params["front_wheel_damp_position"][1].get<cpFloat>()),
                                         params["front_wheel_damp_length"].get<cpFloat>(),
                                         params["front_wheel_damp_stiffness"].get<cpFloat>(),
                                         params["front_wheel_damp_damping"].get<cpFloat>());

    stop_points[2] = cpv(front_wheel_radius * 2.0 * mirror, 1);
    stop_points[3] = cpv(front_wheel_radius * 2.0 * mirror, 0);

    front_wheel_stop = cpPolyShapeNew(car_body,
                                      4,
                                      stop_points.data(),
                                      cpTransformNew(1, 0, 0, 1,
                                                     params["front_wheel_damp_position"][0].get<cpFloat>() *
                                                     mirror - front_wheel_radius * mirror,
                                                     params["front_wheel_damp_position"][1].get<cpFloat>()),
                                      0.0);
    // ******************************************************************************************

    if (drive_type == DRIVE::FR || drive_type == DRIVE::AWD) {
        engines.emplace_back(cpSimpleMotorNew(rear_wheel_body, car_body, 0.0));
    }

    if (drive_type == DRIVE::FF || drive_type == DRIVE::AWD) {
        engines.emplace_back(cpSimpleMotorNew(front_wheel_body, car_body, 0.0));
    }

    // ******************************************************************************************

    cpBodySetPosition(car_body, pos);
    cpBodySetPosition(front_wheel_body, pos + cpv(params["front_wheel_position"][0].get<cpFloat>() * mirror,
                                           params["front_wheel_position"][1].get<cpFloat>()));
    cpBodySetPosition(rear_wheel_body, pos + cpv(params["rear_wheel_position"][0].get<cpFloat>() * mirror,
                                           params["rear_wheel_position"][1].get<cpFloat>()));
}

void Car::move(int direction) {
    if (direction) {

//        def in_air(self):
//        return not (self.point_query_nearest(self.rear_wheel_body.position, self.rear_wheel_radius + 1, pymunk.ShapeFilter(group=self.car_group))
//                    or self.point_query_nearest(self.front_wheel_body.position, self.front_wheel_radius + 1, pymunk.ShapeFilter(group=self.car_group)))

        if (!(cpSpacePointQueryNearest(space_attached,
                                       cpBodyGetPosition(rear_wheel_body), rear_wheel_radius + 1.0, car_filter, nullptr)
              or cpSpacePointQueryNearest(space_attached,
                                          cpBodyGetPosition(front_wheel_body), front_wheel_radius + 1.0, car_filter,
                                          nullptr))) {
//            cpBodySetAngularVelocity(car_body, max_angular_speed * direction);

            cpBodySetTorque(car_body, torque * direction);
        }
    }

    for (auto *engine:engines) {
        cpSimpleMotorSetRate(engine, -max_speed * direction);
    }

}

Car::~Car() {
    for (auto *engine:engines) {
        cpConstraintFree(engine);
    }

    cpConstraintFree(front_wheel_joint);
    cpConstraintFree(front_wheel_dump);
    cpConstraintFree(rear_wheel_joint);
    cpConstraintFree(rear_wheel_dump);

    cpShapeFree(car_shape);
    cpShapeFree(button_shape);
    cpShapeFree(front_wheel_shape);
    cpShapeFree(front_wheel_stop);
    cpShapeFree(rear_wheel_shape);
    cpShapeFree(rear_wheel_stop);

    cpBodyFree(car_body);
    cpBodyFree(front_wheel_body);
    cpBodyFree(rear_wheel_body);
}

void Car::attach(cpSpace *space) {
    // WARNING: ORDER MATTERS!
    cpSpaceAddShape(space, button_shape);
    cpSpaceAddBody(space, car_body);
    cpSpaceAddShape(space, car_shape);
    cpSpaceAddBody(space, rear_wheel_body);
    cpSpaceAddBody(space, front_wheel_body);
    cpSpaceAddShape(space, rear_wheel_shape);
    cpSpaceAddConstraint(space, rear_wheel_joint);
    cpSpaceAddConstraint(space, rear_wheel_dump);
    cpSpaceAddShape(space, rear_wheel_stop);
    cpSpaceAddShape(space, front_wheel_shape);
    cpSpaceAddConstraint(space, front_wheel_joint);
    cpSpaceAddConstraint(space, front_wheel_dump);
    cpSpaceAddShape(space, front_wheel_stop);

    for (auto *engine:engines) {
        cpSpaceAddConstraint(space, engine);
    }
}

void Car::detach(cpSpace *space) {
    cpSpaceRemoveShape(space, button_shape);
    cpSpaceRemoveBody(space, car_body);
    cpSpaceRemoveShape(space, car_shape);
    cpSpaceRemoveBody(space, rear_wheel_body);
    cpSpaceRemoveBody(space, front_wheel_body);
    cpSpaceRemoveShape(space, rear_wheel_shape);
    cpSpaceRemoveConstraint(space, rear_wheel_joint);
    cpSpaceRemoveConstraint(space, rear_wheel_dump);
    cpSpaceRemoveShape(space, rear_wheel_stop);
    cpSpaceRemoveShape(space, front_wheel_shape);
    cpSpaceRemoveConstraint(space, front_wheel_joint);
    cpSpaceRemoveConstraint(space, front_wheel_dump);
    cpSpaceRemoveShape(space, front_wheel_stop);

    for (auto *engine:engines) {
        cpSpaceRemoveConstraint(space, engine);
    }
}

#ifdef REWIND_VIEWER

void draw_poly(RewindClient &rw_client, cpBody *body, cpShape *poly, uint32_t color) {
    int count = cpPolyShapeGetCount(poly);
    const cpVect &first = cpBodyLocalToWorld(body, cpPolyShapeGetVert(poly, 0));
    cpVect prev = first;

    for (int i = 1; i < count; i++) {
        const cpVect &curr = cpBodyLocalToWorld(body, cpPolyShapeGetVert(poly, i));
        rw_client.line(prev.x, prev.y, curr.x, curr.y, color);
        prev = curr;
    }
    rw_client.line(prev.x, prev.y, first.x, first.y, color);
}

void draw_wheel(RewindClient &rw_client, cpBody *wheel_body, cpShape *wheel_shape,
                cpBody *car_body, cpShape *wheel_stop,
                uint32_t wheel_color, uint32_t stop_color, uint32_t line_color, bool square) {

    const cpVect &rear_pos = cpBodyGetPosition(wheel_body);

    double angle = cpBodyGetAngle(wheel_body);
    if (square) {
        draw_poly(rw_client, wheel_body, wheel_shape, wheel_color);
        rw_client.line(rear_pos.x, rear_pos.y, rear_pos.x + 10.0 * cos(angle),
                       rear_pos.y + 10.0 * sin(angle), line_color);
    } else {
        double radius = cpCircleShapeGetRadius(wheel_shape);
        rw_client.circle(rear_pos.x, rear_pos.y, radius, wheel_color);
        rw_client.line(rear_pos.x, rear_pos.y, rear_pos.x + radius * cos(angle),
                       rear_pos.y + radius * sin(angle), line_color);
    }
    draw_poly(rw_client, car_body, wheel_stop, stop_color);
}

#include <math.h>

void Car::draw(RewindClient &rw_client) {

    uint32_t gray = 0x7F7F7F;
    uint32_t dark_gray = 0x333333;
    uint32_t black = 0x000000;
    uint32_t wheel_color = 0x7F7F7F7F;
    uint32_t center_gravity_color = 0xf4ad42;
    uint32_t stop_color = black;
    uint32_t car_color = black;
    uint32_t button_color = player_id % 2 ? 0xFF0000 : 0x0000FF;

//    cpCircleShapeGetOffset(fro)
    const cpVect &car_center = cpBodyLocalToWorld(car_body, cpBodyGetCenterOfGravity(car_body));
    rw_client.circle(car_center.x, car_center.y, 3.0, center_gravity_color);
    rw_client.line(car_center.x, car_center.y - 5.0, car_center.x, car_center.y + 5.0, black);
    rw_client.line(car_center.x - 5.0, car_center.y, car_center.x + 5.0, car_center.y, black);

    draw_wheel(rw_client, rear_wheel_body, rear_wheel_shape, car_body, rear_wheel_stop, wheel_color, stop_color, black,
               squared_wheels);
    draw_wheel(rw_client, front_wheel_body, front_wheel_shape, car_body, front_wheel_stop, wheel_color, stop_color,
               black, squared_wheels);

    draw_poly(rw_client, car_body, car_shape, car_color);
    draw_poly(rw_client, car_body, button_shape, button_color);
}

#endif

void Car::set_from_json(const json &params) {
    car_angle = params[1].get<cpFloat>();
    car_position = cpv(params[0][0].get<cpFloat>(), params[0][1].get<cpFloat>());
    rear_wheel_angle = params[3][2].get<cpFloat>();
    rear_wheel_position = cpv(params[3][0].get<cpFloat>(), params[3][1].get<cpFloat>());
    front_wheel_angle = params[4][2].get<cpFloat>();
    front_wheel_position = cpv(params[4][0].get<cpFloat>(), params[4][1].get<cpFloat>());

    cpBodySetPosition(car_body, car_position);
    cpBodySetAngle(car_body, car_angle);
    cpBodySetPosition(rear_wheel_body, rear_wheel_position);
    cpBodySetAngle(rear_wheel_body, rear_wheel_angle);
    cpBodySetPosition(front_wheel_body, front_wheel_position);
    cpBodySetAngle(front_wheel_body, front_wheel_angle);
}

//#include <iostream>

void Car::update_from_json(const json &params) {
    cpFloat old_car_angle = car_angle;
    cpVect old_car_position = car_position;
    cpFloat old_rear_wheel_angle = rear_wheel_angle;
    cpVect old_rear_wheel_position = rear_wheel_position;
    cpFloat old_front_wheel_angle = front_wheel_angle;
    cpVect old_front_wheel_position = front_wheel_position;

    car_angle = params[1].get<cpFloat>();
    car_position = cpv(params[0][0].get<cpFloat>(), params[0][1].get<cpFloat>());
    rear_wheel_angle = params[3][2].get<cpFloat>();
    rear_wheel_position = cpv(params[3][0].get<cpFloat>(), params[3][1].get<cpFloat>());
    front_wheel_angle = params[4][2].get<cpFloat>();
    front_wheel_position = cpv(params[4][0].get<cpFloat>(), params[4][1].get<cpFloat>());

//    car_angle_speed = (car_angle - old_car_angle) * GAME::SIMULATION_DT_INVERSED;
//    car_speed = cpvmult(car_position - old_car_position, GAME::SIMULATION_DT_INVERSED);
////    std::cerr << "pos_old    "<< old_car_position.x << " - " << old_car_position.y << std::endl;
////    std::cerr << "pos_new    "<< car_position.x << " - " << car_position.y << std::endl;
////    std::cerr << "speeed_simm    "<< cpBodyGetVelocity(car_body).x << " - " << cpBodyGetVelocity(car_body).y << std::endl;
////    std::cerr << "speeed_json    "<< car_speed.x << " - " << car_speed.y << std::endl;
//    rear_wheel_angle_speed = (rear_wheel_angle - old_rear_wheel_angle) * GAME::SIMULATION_DT_INVERSED;
//    rear_wheel_speed = cpvmult(rear_wheel_position - old_rear_wheel_position, GAME::SIMULATION_DT_INVERSED);
//    front_wheel_angle_speed = (front_wheel_angle - old_front_wheel_angle) * GAME::SIMULATION_DT_INVERSED;
//    front_wheel_speed = cpvmult(front_wheel_position - old_front_wheel_position, GAME::SIMULATION_DT_INVERSED);

    car_angle_speed = cpBodyGetAngularVelocity(car_body);
    car_speed = cpBodyGetVelocity(car_body);
    rear_wheel_angle_speed = cpBodyGetAngularVelocity(rear_wheel_body);
    rear_wheel_speed = cpBodyGetVelocity(rear_wheel_body);
    front_wheel_angle_speed = cpBodyGetAngularVelocity(front_wheel_body);
    front_wheel_speed = cpBodyGetVelocity(front_wheel_body);

//    car_force = cpBodyGetForce(car_body);
//    rear_wheel_force=cpBodyGetForce(rear_wheel_body);
//    front_wheel_force=cpBodyGetForce(front_wheel_body);

    reset();
}

void Car::reset() {
    cpBodySetPosition(car_body, car_position);
    cpBodySetAngle(car_body, car_angle);
    cpBodySetPosition(rear_wheel_body, rear_wheel_position);
    cpBodySetAngle(rear_wheel_body, rear_wheel_angle);
    cpBodySetPosition(front_wheel_body, front_wheel_position);
    cpBodySetAngle(front_wheel_body, front_wheel_angle);

    cpBodySetVelocity(car_body, car_speed);
    cpBodySetAngularVelocity(car_body, car_angle_speed);
    cpBodySetVelocity(rear_wheel_body, rear_wheel_speed);
    cpBodySetAngularVelocity(rear_wheel_body, rear_wheel_angle_speed);
    cpBodySetVelocity(front_wheel_body, front_wheel_speed);
    cpBodySetAngularVelocity(front_wheel_body, front_wheel_angle_speed);

//    cpSpaceSetCollisionPersistence()

//    cpBodySetForce(car_body, car_force);
//    cpBodySetForce(rear_wheel_body, rear_wheel_force);
//    cpBodySetForce(front_wheel_body, front_wheel_force);
}
