#ifndef MINI_AI_CUP_3_CAR_H
#define MINI_AI_CUP_3_CAR_H

#include "../chipmunk/include/chipmunk.h"

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
    bool is_attached;
    cpFloat front_wheel_radius, rear_wheel_radius;

    void attach();

public:
    double torque;
    double max_speed;
    int external_id;
    int player_id;

    cpSpace *space;
    cpBody *car_body;
    cpShape *car_shape, *button_shape;

    cpBody *front_wheel_body, *rear_wheel_body;
    cpShape *front_wheel_shape, *rear_wheel_shape;
    cpShape *front_wheel_stop, *rear_wheel_stop;
    cpConstraint *front_wheel_joint, *rear_wheel_joint;
    cpConstraint *front_wheel_dump, *rear_wheel_dump;
    std::vector<cpConstraint *> engines;

    DRIVE::Type drive_type;
    cpGroup car_group;
    cpShapeFilter car_filter;
    cpCollisionType button_collision_type;
    bool squared_wheels;

    Car(const json &params, cpSpace *space_to_attach, double mirror, int player_id);
    virtual ~Car();

    void detach();
    void move(int direction);
    void set_from_json(const json &params);
    void update_from_json(const json &params);
//    void remember_state();
//    void restore_state();

#ifdef REWIND_VIEWER

    void draw(RewindClient &rw_client);

#endif
};


#endif //MINI_AI_CUP_3_CAR_H
