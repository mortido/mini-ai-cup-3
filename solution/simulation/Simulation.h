#ifndef MINI_AI_CUP_3_SIMULATOR_H
#define MINI_AI_CUP_3_SIMULATOR_H

#include <memory>
#include <array>

#include "Map.h"
#include "Car.h"
#include "Deadline.h"
#include <chipmunk/chipmunk.h>

#include "../../../nlohmann/json.hpp"

#ifdef REWIND_VIEWER
#include "../Solution.h"
#endif

using json = nlohmann::json;

#define HEAP_SIZE 1024*1024*16

class Simulation {
private:
    void *heap, *buffer;
    size_t copy_size;
public:
    int saved_tick;
    int sim_tick_index;
    cpSpace *space;
    std::unique_ptr<Map> map;
    std::unique_ptr<Deadline> deadline;
    std::array<std::unique_ptr<Car>, 2> cars;

    Simulation();
    virtual ~Simulation();

    void new_round(const json &params);
    void step();
    void move_car(int player_id, int move);
    void restore();
    void save();
    cpFloat get_closest_point_to_button(int player_id);
    cpFloat get_my_distance_to_enemy_button(int me, int enemy);
    cpFloat get_my_distance_to_enemy_DNISCHE(int me, int enemy);
    cpFloat get_distance_to_deadline(int player_id);

#ifdef LOCAL_RUN
    cpVect car_pos_error;
    void check(int my_player_id, const json &params);
#endif

#ifdef REWIND_VIEWER
    void draw(json &params, int my_player, std::array<Solution, 2> best_solutions);
    RewindClient &rewind;
#endif
};

#endif //MINI_AI_CUP_3_SIMULATOR_H
