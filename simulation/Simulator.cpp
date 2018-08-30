#include "Simulator.h"
#include "../Constants.h"

Simulator::Simulator() : space(cpSpaceNew()) {
    cpSpaceSetGravity(space, cpv(GAME::X_GRAVITY, GAME::Y_GRAVITY));
}

Simulator::~Simulator() {
    space->
    cpSpaceFree(space);
}
