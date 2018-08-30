#ifndef MINI_AI_CUP_3_CAR_H
#define MINI_AI_CUP_3_CAR_H

#include "../../nlohmann/json.hpp"

using json = nlohmann::json;

class Car {
public:
    Car(json params);
};


#endif //MINI_AI_CUP_3_CAR_H
