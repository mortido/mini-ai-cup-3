#include "Map.h"
#include "../Constants.h"

#include <iostream>

Map::~Map() {
    if(is_attached){
        detach();
    }
    for(cpShape *shape:shapes){
        cpShapeFree(shape);
    }
}

Map::Map(const json &params, cpSpace *space_to_attach):is_attached{true} {
    external_id = params["external_id"].get<int>();
    space = space_to_attach;
    cpBody *staticBody = cpSpaceGetStaticBody(space);

    for (auto &segment:params["segments"]) {
        cpShape *ground_segment = cpSegmentShapeNew(staticBody,
                          cpv(segment[0][0].get<double>(), segment[0][1].get<double>()),
                          cpv(segment[1][0].get<double>(), segment[1][1].get<double>()),
                          segment[2].get<double>());
        cpShapeSetFriction(ground_segment, GAME::MAP_FRICTION);
        cpShapeSetElasticity(ground_segment, GAME::MAP_ELASTICITY);
        cpSpaceAddShape(space, ground_segment);
        shapes.push_back(ground_segment);
    }
}

void Map::detach() {
    is_attached = false;
    for(cpShape *shape:shapes){
        cpSpaceRemoveShape(space, shape);
    }
}
