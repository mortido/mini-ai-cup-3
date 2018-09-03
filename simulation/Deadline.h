#ifndef MINI_AI_CUP_3_DEADLINE_H
#define MINI_AI_CUP_3_DEADLINE_H

#include "../chipmunk/include/chipmunk.h"

class Deadline {
public:
    enum Type{
        ASC=0,
        DESC=1
    };

    cpBody *body;
    cpShape *shape;
    Type type;

    Deadline(Type type, cpFloat max_length, cpFloat max_height);
    ~Deadline();
    void detach(cpSpace *space);
    void attach(cpSpace *space);
    void move();
};


#endif //MINI_AI_CUP_3_DEADLINE_H
