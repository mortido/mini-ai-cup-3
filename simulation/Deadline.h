#ifndef MINI_AI_CUP_3_DEADLINE_H
#define MINI_AI_CUP_3_DEADLINE_H

#include <chipmunk/chipmunk.h>

#include "utils.hpp"

#ifdef REWIND_VIEWER
#include "../RewindClient.h"
#endif

class Deadline {
public:
    enum Type{
        ASC=0,
        DESC=1
    };

    cpBody *body, *body_linked;
    cpShape *shape;
    Type type;

    Deadline(Type type, cpFloat max_length, cpFloat max_height);
    ~Deadline();
    void detach(cpSpace *space);
    void attach(cpSpace *space);
    void move();
    void link_to(Deadline *deadline);
    void reset();

#ifdef REWIND_VIEWER
    void draw(RewindClient &rw_client);

#endif
};


#endif //MINI_AI_CUP_3_DEADLINE_H
