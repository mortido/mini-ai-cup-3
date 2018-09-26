#include "Map.h"
#include "../Constants.h"

#include <iostream>
#include <math.h>

#define PI 3.14159265358979323846264338327950288

Map::~Map() {
    for (cpShape *shape:shapes) {
        cpShapeFree(shape);
    }
}

Map::Map(const json &params, cpSpace *space, int car_id) {
    external_id = params["external_id"].get<int>();
    cpBody *staticBody = cpSpaceGetStaticBody(space);

    cpFloat bo = 10.0 - 1.0;
    cpShape *left = cpSegmentShapeNew(staticBody, cpv(-bo, -bo), cpv(-bo, GAME::MAX_HEIGHT + bo), 10.0);
    cpShapeSetSensor(left, true);
    shapes.push_back(left);

    cpShape *top = cpSegmentShapeNew(staticBody, cpv(-bo, GAME::MAX_HEIGHT + bo),
                                     cpv(GAME::MAX_WIDTH + bo, GAME::MAX_HEIGHT + bo), 10.0);
    cpShapeSetSensor(top, true);
    shapes.push_back(top);

    cpShape *right = cpSegmentShapeNew(staticBody, cpv(GAME::MAX_WIDTH + bo, GAME::MAX_HEIGHT + bo),
                                       cpv(GAME::MAX_WIDTH + bo, -bo), 10.0);
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

    for (auto s:shapes) {
        cpShapeSetFilter(s, cpShapeFilterNew(CP_NO_GROUP, 8, CP_ALL_CATEGORIES));
    }

    weights.fill({});

    switch (external_id + car_id * 10) {
        case 21: {
            double val{1};
            int p = 29;
            int u = 60 - p;
            for (int t = 0; t <= p; t++) {
                val = (double) t / p;
                for (int y = 0; y < 80; y++) {
                    weights[60 + t][y] = val;
                    weights[59 - t][y] = val;
                }
            }

            for (int x = 0; x < u; x++) {
                val = (double) x / u;
                for (int y = 0; y < 80; y++) {
                    weights[x][y] = val*(y/80.0);
                    weights[119 - x][y] = val*(y/80.0);
                }
            }
            break;
        }
//        case 12:
        case 22: {
            double m1{0.25};
            double m2{2.0 * (0.5 - m1)};
            double val{0};
            for (int t = 0; t <= 30; t++) {
                val = m1 * t / 30.0;
                for (int y = 0; y < 15; y++) {
                    weights[60 + t][y] = val;
                    weights[59 - t][y] = val;
                }
                val = 1 - val;
                for (int y = 41; y < 80; y++) {
                    weights[60 + t][y] = val;
                    weights[59 - t][y] = val;
                }
            }

            for (int x = 1; x < 30; x++) {
                for (int y = 0; y < 80; y++) {
                    val = 0.5 + m2 * (atan2(y - 40, x)) / PI;
                    weights[29 - x][y] = val;
                    weights[90 + x][y] = val;
                }
            }
            break;
        }
//        case 13:
        case 23: { // PillHill
            double m1{0.25};
            double m2{2.0 * (0.5 - m1)};
            double val{0};
            for (int t = 0; t <= 30; t++) {
                val = m1 * t / 30.0;
                for (int y = 0; y < 19; y++) {
                    weights[60 + t][y] = val;
                    weights[59 - t][y] = val;
                }
                val = 1 - val;
                for (int y = 41; y < 80; y++) {
                    weights[60 + t][y] = val;
                    weights[59 - t][y] = val;
                }
            }

            for (int x = 1; x < 30; x++) {
                for (int y = 0; y < 80; y++) {
                    val = 0.5 + m2 * (atan2(y - 40, x)) / PI;
                    weights[29 - x][y] = val;
                    weights[90 + x][y] = val;
                }
            }

            break;
        }
        case 26: { // IslandHole
//            double val{1};
//            for (int t = 0; t <= 35; t++) {
//                val = t / 35.0;
//                for (int y = 0; y < 80; y++) {
//                    weights[60 + t][y] = val;
//                    weights[59 - t][y] = val;
//                }
//            }
//
//            for (int x = 1; x < 25; x++) {
//                for (int y = 0; y < 80; y++) {
//                    weights[24 - x][y] = 1.0;
//                    weights[95 + x][y] = 1.0;
//                }
//            }
            break;
        }
        case 24: {
            double m1{0.25};
            double m2{2.0 * (0.5 - m1)};
            double val{0};
            for (int t = 0; t <= 30; t++) {
                val = m1 * t / 30.0;
                for (int y = 0; y < 19; y++) {
                    weights[60 + t][y] = val;
                    weights[59 - t][y] = val;
                }
                val = 1 - val;
                for (int y = 41; y < 80; y++) {
                    weights[60 + t][y] = val;
                    weights[59 - t][y] = val;
                }
            }

            for (int x = 1; x < 30; x++) {
                for (int y = 0; y < 80; y++) {
                    val = 0.5 + m2 * (atan2(y - 40, x)) / PI;
                    weights[29 - x][y] = val;
                    weights[90 + x][y] = val;
                }
            }
            break;
        }
        case 25: {
            double m{0.5};
            double val{0};

            for (int x = 10; x < 30; x++) {
                for (int y = 11; y < 80; y++) {
                    val = m * (x - 10.0) / 20.0;
                    weights[119 - x][y] = val;
                    weights[x][y] = val;
                }
            }

            for (int x = 30; x < 90; x++) {
                for (int y = 11; y < 80; y++) {
                    weights[x][y] = m;
                }
            }
            break;
        }
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


//    rw_client.CurrentLayer = 1;
//    for (int x = 0; x < 120; x++) {
//        for (int y = 0; y < 80; y++) {
//            uint32_t alpha = static_cast<uint32_t>(0xFF * weights[x][y]) << (3 * 8);
//            if (alpha) {
//                rw_client.rect(x * 10, y * 10, (x + 1) * 10, (y + 1) * 10, alpha | 0x00FF00);
//            }
//        }
//    }
//    rw_client.CurrentLayer = RewindClient::DEFAULT_LAYER;

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

#endif
