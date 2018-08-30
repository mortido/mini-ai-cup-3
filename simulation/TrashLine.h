#ifndef MINI_AI_CUP_3_TRASHLINE_H
#define MINI_AI_CUP_3_TRASHLINE_H

#include "../chipmunk/include/chipmunk.h"

namespace TL{
    enum Type{
        ASC,
        DESC
    };
}

class TrashLine {
private:
    cpBody *line_body;
    cpShape *line;
    TL::Type type;

public:
    TrashLine();

    void reset();
    void move();
    double get_position();
};


#endif //MINI_AI_CUP_3_TRASHLINE_H
