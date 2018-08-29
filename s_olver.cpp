#include "s_olver.h"

#define NOW high_resolution_clock::now()
#define ELAPSED_TIME duration_cast<duration<double>>(NOW - start_time).count()

Solver::Solver() {

}

void Solver::solve(high_resolution_clock::time_point &start_time) {
    int iteration = 0;

    #define LIMIT ELAPSED_TIME < GA::TIME_LIMIT

    while(LIMIT) {
        if(0 == (iteration % GA::SOLVE_ENEMY_EVERY_N_TURNS)){
            // solve for enemy
        }else{
            // solve for me
        }

        iteration++;
    }
}