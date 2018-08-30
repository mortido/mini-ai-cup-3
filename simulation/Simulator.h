#ifndef MINI_AI_CUP_3_SIMULATOR_H
#define MINI_AI_CUP_3_SIMULATOR_H

#include "Map.h"
#include "Car.h"
#include "TrashLine.h"
#include "../chipmunk/include/chipmunk.h"

class Simulator {
private:
    cpSpace *space;
public:
    Simulator();
    void AddMap(Map &map);
    void AddCar(Car &car);
    void AddTrashLine(TrashLine &trashLine);

    void RemoveMap(Map &map);
    void RemoveCar(Car &car);
    void RemoveTrashLine(TrashLine &trashLine);

    virtual ~Simulator();
};


#endif //MINI_AI_CUP_3_SIMULATOR_H
