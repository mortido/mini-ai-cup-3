#ifndef MINI_AI_CUP_3_CAR_H
#define MINI_AI_CUP_3_CAR_H

#include "../chipmunk/include/chipmunk.h"

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
    cpSpace *space;
    cpBody *car_body, *front_wheel_body, *rear_wheel_body;
    cpShape *car_shape, *button_shape, *front_wheel_shape, *rear_wheel_shape;
    DRIVE::Type drive_type;
    int car_group, button_collision_type;


public:
    double max_angular_speed;
    double max_speed;
    int external_id;

    Car(const json &params, cpSpace *space_to_attach, double position, int player_id);
    virtual ~Car();

    void move(int direction);
};


#endif //MINI_AI_CUP_3_CAR_H
