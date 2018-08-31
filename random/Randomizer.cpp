#include "Randomizer.h"
#include "../Constants.h"

std::unique_ptr<Randomizer> Randomizer::instance;

Randomizer::Randomizer() : rng(2707),
                           probability(0.0, 1.0),
//                           moves(0, GAME::MOVES_COUNT),
                           moves(-1, 2), // directions
                           crossover(1, GA::DEPTH-1),
                           mutate(0, GA::DEPTH),
                           parent(0, GA::POPULATION_SIZE),
                           coin(0, 2) {

}

void Randomizer::init() {
    instance = std::unique_ptr<Randomizer>(new Randomizer());
}

double Randomizer::GetProbability() {
    return instance->probability(instance->rng);
}

int Randomizer::GetRandomMove() {
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
