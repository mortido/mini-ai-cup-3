#ifndef MINI_AI_CUP_3_RANDOMIZER_H
#define MINI_AI_CUP_3_RANDOMIZER_H

#include "pcg_random.hpp"
#include <random>

class Randomizer{
private:
    pcg32 rng;
    static std::unique_ptr<Randomizer> instance;
    std::uniform_real_distribution<double> probability;
    std::uniform_int_distribution<int> moves;
    std::uniform_int_distribution<int> crossover;
    std::uniform_int_distribution<int> mutate;
    std::uniform_int_distribution<int> parent;
    std::uniform_int_distribution<int> coin;
    Randomizer();
public:

    static void init();
    static inline double GetProbability();
    static inline int GetRandomMove();
    static inline int GetCrossoverPoint();
    static inline int GetMutatePoint();
    static inline int GetRandomParent();
    static inline int FlipCoin();
};

#endif //MINI_AI_CUP_3_RANDOMIZER_H
