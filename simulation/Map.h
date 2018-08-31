#ifndef MINI_AI_CUP_3_MAP_H
#define MINI_AI_CUP_3_MAP_H

#include <vector>

#include "../chipmunk/include/chipmunk.h"
#include "../../nlohmann/json.hpp"

using json = nlohmann::json;

class Map {
private:
    bool is_attached;
    cpSpace *space;
    std::vector<cpShape*> shapes;

public:
    int external_id;

    Map(const json &params, cpSpace *space_to_attach);
    virtual ~Map();

    void detach();
};


#endif //MINI_AI_CUP_3_MAP_H
