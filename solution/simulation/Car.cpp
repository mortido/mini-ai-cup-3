#include "Car.h"
#include "../Constants.h"
#include <vector>
#include <array>

Car::Car(const json &params, cpSpace *_space, double mirror, int _player_id,
         cpVect pos) : space_attached{_space}, player_id{_player_id}, alive{true} {

    car_group = static_cast<cpGroup>(player_id + 1);
    button_collision_type = static_cast<cpCollisionType>((player_id + 1) * 10);
    car_category = static_cast<cpBitmask>(1 << player_id);

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
    car_filter = cpShapeFilterNew(car_group, car_category, CP_ALL_CATEGORIES);
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

    rear_wheel_groove = cpGrooveJointNew(car_body, rear_wheel_body,
                                         cpv(params["rear_wheel_damp_position"][0].get<cpFloat>() * mirror,
                                             params["rear_wheel_damp_position"][1].get<cpFloat>() -
                                             params["rear_wheel_groove_offset"].get<cpFloat>()),
                                         cpv(params["rear_wheel_damp_position"][0].get<cpFloat>() * mirror,
                                             params["rear_wheel_damp_position"][1].get<cpFloat>() -
                                             1.5 * params["rear_wheel_damp_length"].get<cpFloat>()),
                                         cpvzero);

    rear_wheel_dump = cpDampedSpringNew(rear_wheel_body,
                                        car_body,
                                        cpvzero, cpv(params["rear_wheel_damp_position"][0].get<cpFloat>() * mirror,
                                                     params["rear_wheel_damp_position"][1].get<cpFloat>()),
                                        params["rear_wheel_damp_length"].get<cpFloat>(),
                                        params["rear_wheel_damp_stiffness"].get<cpFloat>(),
                                        params["rear_wheel_damp_damping"].get<cpFloat>());

    if (drive_type == DRIVE::FR || drive_type == DRIVE::AWD) {
        engines.emplace_back(cpSimpleMotorNew(rear_wheel_body, car_body, 0.0));
    }

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

    front_wheel_groove = cpGrooveJointNew(car_body, front_wheel_body,
                                          cpv(params["front_wheel_damp_position"][0].get<cpFloat>() * mirror,
                                              params["front_wheel_damp_position"][1].get<cpFloat>() -
                                              params["front_wheel_groove_offset"].get<cpFloat>()),
                                          cpv(params["front_wheel_damp_position"][0].get<cpFloat>() * mirror,
                                              params["front_wheel_damp_position"][1].get<cpFloat>() -
                                              1.5 * params["front_wheel_damp_length"].get<cpFloat>()),
                                          cpvzero);

    front_wheel_dump = cpDampedSpringNew(front_wheel_body,
                                         car_body,
                                         cpvzero, cpv(params["front_wheel_damp_position"][0].get<cpFloat>() * mirror,
                                                      params["front_wheel_damp_position"][1].get<cpFloat>()),
                                         params["front_wheel_damp_length"].get<cpFloat>(),
                                         params["front_wheel_damp_stiffness"].get<cpFloat>(),
                                         params["front_wheel_damp_damping"].get<cpFloat>());


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

    cpConstraintFree(front_wheel_groove);
    cpConstraintFree(front_wheel_dump);
    cpConstraintFree(rear_wheel_groove);
    cpConstraintFree(rear_wheel_dump);

    cpShapeFree(car_shape);
    cpShapeFree(button_shape);
    cpShapeFree(front_wheel_shape);
    cpShapeFree(rear_wheel_shape);

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
    cpSpaceAddConstraint(space, rear_wheel_groove);
    cpSpaceAddConstraint(space, rear_wheel_dump);
    cpSpaceAddShape(space, front_wheel_shape);
    cpSpaceAddConstraint(space, front_wheel_groove);
    cpSpaceAddConstraint(space, front_wheel_dump);

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
    cpSpaceRemoveConstraint(space, rear_wheel_groove);
    cpSpaceRemoveConstraint(space, rear_wheel_dump);
    cpSpaceRemoveShape(space, front_wheel_shape);
    cpSpaceRemoveConstraint(space, front_wheel_groove);
    cpSpaceRemoveConstraint(space, front_wheel_dump);

    for (auto *engine:engines) {
        cpSpaceRemoveConstraint(space, engine);
    }
}

void Car::detach_shapes(cpSpace *space) {
    cpSpaceRemoveShape(space, button_shape);
    cpSpaceRemoveShape(space, car_shape);
    cpSpaceRemoveShape(space, rear_wheel_shape);
    cpSpaceRemoveShape(space, front_wheel_shape);
}

void Car::detach_bodies(cpSpace *space) {
    cpSpaceRemoveBody(space, car_body);
    cpSpaceRemoveBody(space, rear_wheel_body);
    cpSpaceRemoveBody(space, front_wheel_body);
}

void Car::detach_constraints(cpSpace *space) {
    cpSpaceRemoveConstraint(space, rear_wheel_groove);
    cpSpaceRemoveConstraint(space, rear_wheel_dump);
    cpSpaceRemoveConstraint(space, front_wheel_groove);
    cpSpaceRemoveConstraint(space, front_wheel_dump);

    for (auto *engine:engines) {
        cpSpaceRemoveConstraint(space, engine);
    }
}

#ifdef REWIND_VIEWER

void draw_wheel(RewindClient &rw_client, cpBody *wheel_body, cpShape *wheel_shape,
                uint32_t wheel_color, uint32_t line_color, bool square) {

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
}

void Car::draw(RewindClient &rw_client, bool shadow) {


    uint32_t car_lines = 0xFF000000;
    uint32_t wheel_color = 0xFF7F7F7F;
    uint32_t center_gravity_color = 0xFFf4ad42;
    uint32_t car_color = 0xFF000000;
    uint32_t button_color = player_id % 2 ? 0xFFFF0000 : 0xFF0000FF;

    if (shadow) {
        car_lines &= 0x10FFFFFF;
        wheel_color &= 0x10FFFFFF;
        center_gravity_color &= 0x10FFFFFF;
        car_color &= 0x10FFFFFF;
        button_color &= 0x10FFFFFF;

        car_lines = 0xAAAAAA;
        car_color = 0xAAAAAA;
        button_color = player_id % 2 ? 0xFF550000 : 0xFF0000AA;
    }

//    cpCircleShapeGetOffset(fro)
    const cpVect &car_center = cpBodyLocalToWorld(car_body, cpBodyGetCenterOfGravity(car_body));
    rw_client.circle(car_center.x, car_center.y, 3.0, center_gravity_color);
    rw_client.line(car_center.x, car_center.y - 5.0, car_center.x, car_center.y + 5.0, car_lines);
    rw_client.line(car_center.x - 5.0, car_center.y, car_center.x + 5.0, car_center.y, car_lines);

    draw_wheel(rw_client, rear_wheel_body, rear_wheel_shape, wheel_color, car_lines,
               squared_wheels);
    draw_wheel(rw_client, front_wheel_body, front_wheel_shape, wheel_color,
               car_lines, squared_wheels);

    draw_poly(rw_client, car_body, car_shape, car_color);
    draw_poly(rw_client, car_body, button_shape, button_color);
}

#endif
