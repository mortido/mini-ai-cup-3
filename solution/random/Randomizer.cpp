#include "Randomizer.h"
#include "../Constants.h"

std::unique_ptr<Randomizer> Randomizer::instance;

#ifdef OPTIMIZATION_RUN
Randomizer::Randomizer(int seed) : rng(seed),
                           probability(0.0, 1.0),
//                           moves(0, GAME::MOVES_COUNT),
                           moves(-1, GAME::MOVES_COUNT- 2), // directions
                           crossover(1, GA::DEPTH-2),
                           mutate(0, GA::DEPTH-1),
                           parent(0, GA::POPULATION_SIZE-1),
                           coin(0, 2) {

}

void Randomizer::init(int seed) {
    instance = std::unique_ptr<Randomizer>(new Randomizer(seed));
}
#else
Randomizer::Randomizer() : rng(2707),
                           probability(0.0, 1.0),
//                           moves(0, GAME::MOVES_COUNT),
                           moves(-1, GAME::MOVES_COUNT- 2), // directions
                           crossover(1, GA::DEPTH-2),
                           mutate(0, GA::DEPTH-1),
                           parent(0, GA::POPULATION_SIZE-1),
                           coin(0, 1) {

}

void Randomizer::init() {
    instance = std::unique_ptr<Randomizer>(new Randomizer());
}
#endif

double Randomizer::GetProbability() {
    return instance->probability(instance->rng);
}

int Randomizer::GetRandomMove() {
//    if(instance->coin(instance->rng)){
//        return 1;
//    }
//    return -1;

    return instance->moves(instance->rng);
}

int Randomizer::GetCrossoverPoint() {
    return instance->crossover(instance->rng);
}

int Randomizer::FlipCoin() {
    return instance->coin(instance->rng);
}

int Randomizer::GetMutatePoint() {
    return instance->mutate(instance->rng);
}

int Randomizer::GetRandomParent() {
    return  instance->parent(instance->rng);
}
