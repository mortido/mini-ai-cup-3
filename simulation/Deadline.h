#ifndef MINI_AI_CUP_3_DEADLINE_H
#define MINI_AI_CUP_3_DEADLINE_H

#ifdef REWIND_VIEWER
#include "../RewindClient.h"
#endif

#include <chipmunk/chipmunk.h>

class Deadline {
public:
    enum Type{
        ASC=0,
        DESC=1
    };

    cpBody *body;
    cpShape *shape;
    cpVect position;
    Type type;

    Deadline(Type type, cpFloat max_length, cpFloat max_height);
    ~Deadline();
    void detach(cpSpace *space);
    void attach(cpSpace *space);
    void move();
    void copy_from(Deadline *deadline);
    void reset();

#ifdef REWIND_VIEWER
    void draw(RewindClient &rw_client);

#endif
};


#endif //MINI_AI_CUP_3_DEADLINE_H
