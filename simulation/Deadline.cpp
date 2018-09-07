#include "Deadline.h"

Deadline::~Deadline() {
    cpShapeFree(shape);
    cpBodyFree(body);
}

Deadline::Deadline(Deadline::Type _type, cpFloat max_length, cpFloat max_height) : type{_type}, body_linked{nullptr} {
    body = cpBodyNewKinematic();
    shape = cpSegmentShapeNew(body, cpv(0.0, 0.0), cpv(max_length, 0.0), 2.0);
    cpShapeSetSensor(shape, static_cast<cpBool>(true));
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

void Deadline::reset() {
    copy_body_position(body, body_linked);
}

void Deadline::link_to(Deadline *deadline) {
    body_linked = deadline->body;
    deadline->shape->userData=shape;
}

