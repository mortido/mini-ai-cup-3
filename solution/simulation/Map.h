#ifndef MINI_AI_CUP_3_MAP_H
#define MINI_AI_CUP_3_MAP_H

#include <vector>

#ifdef REWIND_VIEWER
#include "../RewindClient.h"
#endif

#include <chipmunk/chipmunk.h>
#include "../../../nlohmann/json.hpp"

using json = nlohmann::json;

class Map {
private:
    std::vector<cpShape*> shapes;

public:
    int external_id;

    Map(const json &params, cpSpace *space);
    virtual ~Map();
    void detach(cpSpace *space);
    void attach(cpSpace *space);

#ifdef REWIND_VIEWER
    void draw(RewindClient &rw_client);

#endif
};


#endif //MINI_AI_CUP_3_MAP_H
