#ifndef MINI_AI_CUP_3_CAR_H
#define MINI_AI_CUP_3_CAR_H

#include <chipmunk/chipmunk.h>

#ifdef REWIND_VIEWER
#include "../RewindClient.h"
#endif

#include "../../nlohmann/json.hpp"

using json = nlohmann::json;

namespace DRIVE {
    enum Type {
        FF = 1,
        FR = 2,
        AWD = 3
    };
}

class Car {
private:



public:
    cpFloat torque;
    cpFloat max_speed;
    int external_id;
    int player_id;
    bool alive;

    cpSpace *space_attached;
    cpBody *car_body;
    cpShape *car_shape, *button_shape;

    cpBody *front_wheel_body, *rear_wheel_body;
    cpShape *front_wheel_shape, *rear_wheel_shape;
    cpConstraint *front_wheel_groove, *rear_wheel_groove;
    cpConstraint *front_wheel_dump, *rear_wheel_dump;
    std::vector<cpConstraint *> engines;

    DRIVE::Type drive_type;
    cpGroup car_group;
    cpShapeFilter car_filter;
    cpCollisionType button_collision_type;
    bool squared_wheels;
    cpFloat front_wheel_radius, rear_wheel_radius;

    // STATE *****************
    cpFloat car_angle, rear_wheel_angle, front_wheel_angle;
    cpFloat car_angle_speed, rear_wheel_angle_speed, front_wheel_angle_speed;
    cpVect car_position, rear_wheel_position, front_wheel_position;
    cpVect car_speed, rear_wheel_speed, front_wheel_speed;

    Car(const json &params, cpSpace *_space, double mirror, int player_id, cpVect pos);
    virtual ~Car();

    void attach(cpSpace *space);
    void detach(cpSpace *space);
    void detach_shapes(cpSpace *space);
    void detach_bodies(cpSpace *space);
    void detach_constraints(cpSpace *space);

    void move(int direction);
    void copy_from(Car *car);
    void reset();

#ifdef REWIND_VIEWER

    void draw(RewindClient &rw_client);

#endif
};


#endif //MINI_AI_CUP_3_CAR_H
