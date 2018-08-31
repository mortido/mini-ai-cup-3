//#include "Car.h"
//
//Car::~Car() {
//
//}
#include "Car.h"

Car::Car(const json &params, cpSpace *space_to_attach, double mirror, int player_id) {

    car_group = player_id;
    button_collision_type = player_id * 10;

    max_angular_speed = params["max_angular_speed"].get<double>();
    max_speed = params["max_speed"].get<double>();
    external_id = params["external_id"].get<int>();
    drive_type = params["drive"].get<DRIVE::Type>();

//    params["car_body_poly"]:[  ],
//
//            params["button_poly"]:[  ],
//
//            params["car_body_mass"]:200,
//            params["car_body_friction"]:0.9,
//            params["car_body_elasticity"]:0.5,
//
//
//
//            params["squared_wheels"] // optional...
//
//            params["rear_wheel_radius"]:12,
//            params["rear_wheel_mass"]:50,
//            params["rear_wheel_position"]:[  ],
//            params["rear_wheel_friction"]:1,
//            params["rear_wheel_elasticity"]:0.8,
//            params["rear_wheel_joint"]:[  ],
//            params["rear_wheel_damp_position"]:[  ],
//            params["rear_wheel_damp_length"]:25,
//            params["rear_wheel_damp_stiffness"]:50000.0,
//            params["rear_wheel_damp_damping"]:3000.0,
//
//            params["front_wheel_radius"]:12,
//            params["front_wheel_mass"]:5,
//            params["front_wheel_position"]:[  ],
//            params["front_wheel_friction"]:1,
//            params["front_wheel_elasticity"]:0.8,
//            params["front_wheel_joint"]:[  ],
//            params["front_wheel_damp_position"]:[  ],
//            params["front_wheel_damp_length"]:25,
//            params["front_wheel_damp_stiffness"]:60000.0,
//            params["front_wheel_damp_damping"]:900.0
}

void Car::move(int direction) {
    if(direction){
        max_angular_speed * direction;
    }

    max_speed * direction;
}

Car::~Car() {

}
