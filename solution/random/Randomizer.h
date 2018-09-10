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
#ifdef OPTIMIZATION_RUN
    explicit Randomizer(int seed);
#else
    Randomizer();
#endif
public:
#ifdef OPTIMIZATION_RUN
    static void init(int seed);
#else
    static void init();
#endif

    static double GetProbability();
    static int GetRandomMove();
    static int GetCrossoverPoint();
    static int GetMutatePoint();
    static int GetRandomParent();
    static int FlipCoin();
};

#endif //MINI_AI_CUP_3_RANDOMIZER_H
