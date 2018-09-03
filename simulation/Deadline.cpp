#include "Deadline.h"

Deadline::~Deadline() {
    cpShapeFree(shape);
    cpBodyFree(body);
}

Deadline::Deadline(Deadline::Type _type, cpFloat max_length, cpFloat max_height) : type{_type} {
    body = cpBodyNewKinematic();
    shape = cpSegmentShapeNew(body, cpv(0.0, 0.0), cpv(max_length, 0.0), 2.0);
    cpShapeSetSensor(shape, static_cast<cpBool>(true));
    if (type == ASC) {
        cpBodySetPosition(body, cpv(0.0, 10.0));
    } else {
        cpBodySetPosition(body, cpv(0.0, max_height - 10.0));
    }
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

