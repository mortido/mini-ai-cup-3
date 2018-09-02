#include "Map.h"
#include "../Constants.h"

#include <iostream>

Map::~Map() {
    if (is_attached) {
        detach();
    }
    for (cpShape *shape:shapes) {
        cpShapeFree(shape);
    }
}

Map::Map(const json &params, cpSpace *space_to_attach) : is_attached{true}, space{space_to_attach} {
    external_id = params["external_id"].get<int>();
    cpBody *staticBody = cpSpaceGetStaticBody(space);

    for (auto &segment:params["segments"]) {
        cpShape *ground_segment = cpSegmentShapeNew(staticBody,
                                                    cpv(segment[0][0].get<cpFloat>(), segment[0][1].get<cpFloat>()),
                                                    cpv(segment[1][0].get<cpFloat>(), segment[1][1].get<cpFloat>()),
                                                    segment[2].get<cpFloat>());
        cpShapeSetFriction(ground_segment, GAME::MAP_FRICTION);
        cpShapeSetElasticity(ground_segment, GAME::MAP_ELASTICITY);
        cpSpaceAddShape(space, ground_segment);
        shapes.push_back(ground_segment);
    }
}

void Map::detach() {
    is_attached = false;
    for (cpShape *shape:shapes) {
        cpSpaceRemoveShape(space, shape);
    }
}

#ifdef REWIND_VIEWER

void Map::draw(RewindClient &rw_client) {
    uint32_t map_color = 0x263f31;
    for (cpShape *shape:shapes) {
        const cpVect &n = cpSegmentShapeGetNormal(shape);
        const cpVect &a = cpSegmentShapeGetA(shape);
        const cpVect &b = cpSegmentShapeGetB(shape);
        const double r = cpSegmentShapeGetRadius(shape);

        const cpVect &a1 = a - n * r;
        const cpVect &b1 = b - n * r;
        const cpVect &a2 = a + n * r;
        const cpVect &b2 = b + n * r;
        rw_client.line(a1.x, a1.y, b1.x, b1.y, map_color);
        rw_client.line(a2.x, a2.y, b2.x, b2.y, map_color);
    }

}

#endif
