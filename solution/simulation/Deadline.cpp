#include "Deadline.h"

Deadline::~Deadline() {
    cpShapeFree(shape);
    cpBodyFree(body);
}

Deadline::Deadline(Deadline::Type _type, cpFloat max_length, cpFloat max_height) : type{_type}{
    body = cpBodyNewKinematic();

    std::array<cpVect, 4> verts {
        cpv(0.0, 2.0),
        cpv(max_length, 2),
        cpv(max_length, -max_height),
        cpv(0, -max_height),
    };
//    pymunk.Poly(self.line_body, [(0, 2), (max_length, 2), (max_length, -max_height), (0, -max_height)])
//    shape = cpSegmentShapeNew(body, cpv(0.0, 0.0), cpv(max_length, 0.0), 2.0);
    shape = cpPolyShapeNew(body, 4, verts.data(),cpTransformIdentity, 0);
    cpShapeSetSensor(shape, static_cast<cpBool>(true));
    cpShapeSetFilter(shape, cpShapeFilterNew(CP_NO_GROUP, 8,CP_ALL_CATEGORIES));
    cpVect position;
    if (type == ASC) {
        position = cpv(0.0, 10.0);
    } else {
        position = cpv(0.0, max_height - 10.0);
    }
    cpBodySetPosition(body, position);
}

void Deadline::attach(cpSpace *space) {
    cpSpaceAddShape(space, shape);
}

void Deadline::detach(cpSpace *space) {
    cpSpaceRemoveShape(space, shape);
}

void Deadline::move() {
    cpVect pos = cpBodyGetPosition(body);
    if (type == ASC) {
        pos.y += 0.5;
    } else {
        pos.y -= 0.5;
    }
    cpBodySetPosition(body, pos);
}

#ifdef REWIND_VIEWER

void Deadline::draw(RewindClient &rw_client) {
    cpVect pos = cpBodyGetPosition(body);

    const cpVect &n = cpSegmentShapeGetNormal(shape);
    const cpVect &a = cpSegmentShapeGetA(shape);
    const cpVect &b = cpSegmentShapeGetB(shape);
    const double r = cpSegmentShapeGetRadius(shape);

    const cpVect &a1 = a - n * r;
    const cpVect &b1 = b - n * r;
    const cpVect &a2 = a + n * r;
    const cpVect &b2 = b + n * r;
    rw_client.line(a1.x, a1.y + pos.y, b1.x, b1.y + pos.y, 0xff6344);
    rw_client.line(a2.x, a2.y + pos.y, b2.x, b2.y + pos.y, 0xff6344);
}

#endif
