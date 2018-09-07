#include "Map.h"
#include "../Constants.h"

#include <iostream>

Map::~Map() {
    for (cpShape *shape:shapes) {
        cpShapeFree(shape);
    }
}

Map::Map(const json &params, cpSpace *space) {
    external_id = params["external_id"].get<int>();
    cpBody *staticBody = cpSpaceGetStaticBody(space);

    cpFloat bo = 10.0 - 1.0;
    cpShape *left = cpSegmentShapeNew(staticBody, cpv(-bo, -bo), cpv(-bo, GAME::MAX_HEIGHT + bo), 10.0);
    cpShapeSetSensor(left, true);
    shapes.push_back(left);

    cpShape *top = cpSegmentShapeNew(staticBody, cpv(-bo, GAME::MAX_HEIGHT + bo), cpv(GAME::MAX_WIDTH + bo, GAME::MAX_HEIGHT + bo), 10.0);
    cpShapeSetSensor(top, true);
    shapes.push_back(top);

    cpShape *right = cpSegmentShapeNew(staticBody, cpv(GAME::MAX_WIDTH + bo, GAME::MAX_HEIGHT + bo), cpv(GAME::MAX_WIDTH + bo, -bo), 10.0);
    cpShapeSetSensor(right, true);
    shapes.push_back(right);

    cpShape *bottom = cpSegmentShapeNew(staticBody, cpv(GAME::MAX_WIDTH + bo, -bo), cpv(-bo, -bo), 10.0);
    cpShapeSetSensor(bottom, true);
    shapes.push_back(bottom);

    for (auto &segment:params["segments"]) {
        cpShape *ground_segment = cpSegmentShapeNew(staticBody,
                                                    cpv(segment[0][0].get<cpFloat>(), segment[0][1].get<cpFloat>()),
                                                    cpv(segment[1][0].get<cpFloat>(), segment[1][1].get<cpFloat>()),
                                                    segment[2].get<cpFloat>());
        cpShapeSetFriction(ground_segment, GAME::MAP_FRICTION);
        cpShapeSetElasticity(ground_segment, GAME::MAP_ELASTICITY);
        shapes.push_back(ground_segment);
    }
}

void Map::attach(cpSpace *space) {
//    is_attached = false;
    for (cpShape *shape:shapes) {
        cpSpaceAddShape(space, shape);
    }
}

void Map::detach(cpSpace *space) {
//    is_attached = false;
    for (cpShape *shape:shapes) {
        cpSpaceRemoveShape(space, shape);
    }
}

#ifdef REWIND_VIEWER

void draw_segment(RewindClient &rw_client, cpShape *shape, uint32_t color) {
    const cpVect &n = cpSegmentShapeGetNormal(shape);
    const cpVect &a = cpSegmentShapeGetA(shape);
    const cpVect &b = cpSegmentShapeGetB(shape);
    const double r = cpSegmentShapeGetRadius(shape);

    const cpVect &a1 = a - n * r;
    const cpVect &b1 = b - n * r;
    const cpVect &a2 = a + n * r;
    const cpVect &b2 = b + n * r;
    rw_client.line(a1.x, a1.y, b1.x, b1.y, color);
    rw_client.line(a2.x, a2.y, b2.x, b2.y, color);
    rw_client.line(a1.x, a1.y, a2.x, a2.y, color);
    rw_client.line(b1.x, b1.y, b2.x, b2.y, color);
}

void Map::draw(RewindClient &rw_client) {
    uint32_t map_color = 0x263f31;
    for (cpShape *shape:shapes) {
        draw_segment(rw_client, shape, map_color);
//        const cpVect &n = cpSegmentShapeGetNormal(shape);
//        const cpVect &a = cpSegmentShapeGetA(shape);
//        const cpVect &b = cpSegmentShapeGetB(shape);
//        const double r = cpSegmentShapeGetRadius(shape);
//
//        const cpVect &a1 = a - n * r;
//        const cpVect &b1 = b - n * r;
//        const cpVect &a2 = a + n * r;
//        const cpVect &b2 = b + n * r;
//        rw_client.line(a1.x, a1.y, b1.x, b1.y, map_color);
//        rw_client.line(a2.x, a2.y, b2.x, b2.y, map_color);
    }

}

#include <chipmunk/chipmunk_structs.h>

void Map::link_to(Map *map) {
    for (int i=0;i<shapes.size();i++){
        map->shapes[i]->userData = shapes[i];
    }
}

#endif
