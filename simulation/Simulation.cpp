#include "Simulation.h"
#include "../Constants.h"

Simulation::Simulation() : space{nullptr}, sim_tick_index{0}
#ifdef REWIND_VIEWER
                           ,rewind(RewindClient::instance())
#endif
                           {
}

Simulation::~Simulation() {
    cpSpaceFree(space);
}

static cpBool kill_car_on_button_press(cpArbiter *arb, cpSpace *space, bool *alive){
    *alive = false;
    return cpFalse;
}

void Simulation::new_round(const json &params) {
#ifdef LOCAL_RUN
    car_pos_error.x=0.0;
    car_pos_error.y=0.0;
#endif

if(space){
    map->detach(space);
    deadline->detach(space);
//        cars[0]->detach_shapes(space);
//        cars[1]->detach_shapes(space);
//        cars[0]->detach_bodies(space);
//        cars[1]->detach_bodies(space);
//        cars[0]->detach_constraints(space);
//        cars[1]->detach_constraints(space);
    cars[0]->detach(space);
    cars[1]->detach(space);
}else{
    space = cpSpaceNew();
    cpSpaceSetGravity(space, GAME::GRAVITY); // 0 -700
    cpSpaceSetDamping(space, GAME::DAMPING); // 0.85
//        cpSpaceSetCollisionPersistence(space, 0);
}

    map.reset(new Map(params["proto_map"], space));
    deadline.reset(new Deadline(Deadline::ASC, 1800, 800));

    cars[0].reset(new Car(params["proto_car"], space, 1.0, 0, GAME::LEFT_CAR_POS)); // 300-300
    cpCollisionHandler *ch1 = cpSpaceAddWildcardHandler(space, cars[0]->button_collision_type);
    ch1->beginFunc = (cpCollisionBeginFunc) kill_car_on_button_press;
    ch1->userData = &(cars[0]->alive);

    cars[1].reset(new Car(params["proto_car"], space, -1.0, 1, GAME::RIGHT_CAR_POS)); // 900-300
    cpCollisionHandler *ch2 = cpSpaceAddWildcardHandler(space, cars[1]->button_collision_type);
    ch2->beginFunc = (cpCollisionBeginFunc) kill_car_on_button_press;
    ch2->userData = &(cars[1]->alive);

    map->attach(space);
    deadline->attach(space);
    cars[0]->attach(space);
    cars[1]->attach(space);

    sim_tick_index = 0;
}

void Simulation::step() {
    if (GAME::TICK_TO_DEADLINE - sim_tick_index < 1) {
        deadline->move();
    }

    cpSpaceStep(space, GAME::SIMULATION_DT);
    sim_tick_index++;
}

#ifdef REWIND_VIEWER

void Simulation::draw(json &params) {
//    map->draw(rewind);

    rewind.circle(params["my_car"][3][0].get<double>(), params["my_car"][3][1].get<double>(), cars[0]->rear_wheel_radius, 0x3FCC0000);
    rewind.circle(params["my_car"][4][0].get<double>(), params["my_car"][4][1].get<double>(), cars[0]->front_wheel_radius, 0x3FCC0000);

    cars[0]->draw(rewind);
    cars[1]->draw(rewind);
    deadline->draw(rewind);
//    rewind.end_frame();
}

#endif

cpFloat Simulation::get_closest_point_to_button(int player_id) {
//    cpShape *
//    cpSpacePointQueryNearest(cpSpace *space, cpVect point, cpFloat maxDistance, cpShapeFilter filter, cpPointQueryInfo *out)
//
//    cpPointQueryInfo queryInfo;
//
//    cpSpacePointQueryNearest(space, , 500.0, &queryInfo);
    return 0;
}

#ifdef LOCAL_RUN

void Simulation::check(int my_player_id, const json &params) {
    car_pos_error.x += abs(cpBodyGetPosition(cars[my_player_id]->car_body).x - params["my_car"][0][0].get<cpFloat>());
    car_pos_error.y += abs(cpBodyGetPosition(cars[my_player_id]->car_body).y - params["my_car"][0][1].get<cpFloat>());
}

#endif

void Simulation::move_car(int player_id, int move) {
    cars[player_id]->move(move);
}

void Simulation::link_to(const Simulation &sim) {
    map->link_to(sim.map.get());
    deadline->link_to(sim.deadline.get());
    cars[0] ->link_to(sim.cars[0].get());
    cars[1] ->link_to(sim.cars[1].get());
    linked_space = sim.space;
}


#include <utility>
#include <chipmunk/chipmunk_structs.h>
extern "C"{
#include <chipmunk/chipmunk_private.h>
}

#include <iostream>

static cpBool cachedArbitersRemoveAll(cpArbiter *arb, cpSpace *space){
    arb->contacts = NULL;
    arb->count = 0;
    cpArrayPush(space->pooledArbiters, arb);
    return cpFalse;
}

static void *
cpSpaceArbiterSetTrans(cpShape **shapes, cpSpace *space)
{
    if(space->pooledArbiters->num == 0){
        // arbiter pool is exhausted, make more
        int count = CP_BUFFER_BYTES/sizeof(cpArbiter);
        cpAssertHard(count, "Internal Error: Buffer size too small.");

        cpArbiter *buffer = (cpArbiter *) cpcalloc(1, CP_BUFFER_BYTES);
        cpArrayPush(space->allocatedBuffers, buffer);

        for(int i=0; i<count; i++) cpArrayPush(space->pooledArbiters, buffer + i);
    }

    return cpArbiterInit((cpArbiter *)cpArrayPop(space->pooledArbiters), shapes[0], shapes[1]);
}

static inline cpCollisionHandler *
cpSpaceLookupHandler(cpSpace *space, cpCollisionType a, cpCollisionType b, cpCollisionHandler *defaultValue)
{
    cpCollisionType types[] = {a, b};
    cpCollisionHandler *handler = (cpCollisionHandler *)cpHashSetFind(space->collisionHandlers, CP_HASH_PAIR(a, b), types);
    return (handler ? handler : defaultValue);
}

static void copyCachedArbiter(cpArbiter *arb, std::pair<cpSpace *, cpSpace*> spaces){
    const cpShape *shape_pair[] = {(cpShape*)arb->a->userData, (cpShape*)arb->b->userData};

//    std::cerr << "ARB: " << arb->a << "->" << arb->a->userData << " ; ";
//    std::cerr << arb->b << "->" << arb->b->userData << std::endl;

    cpHashValue arbHashID = CP_HASH_PAIR((cpHashValue)arb->a->userData, (cpHashValue)arb->b->userData);
    cpArbiter *shadow_arb = (cpArbiter *)cpHashSetInsert(spaces.first->cachedArbiters, arbHashID, shape_pair, (cpHashSetTransFunc)cpSpaceArbiterSetTrans, spaces.first);

    // Shift arbiter time stamp (don't update stamp on space as it is used for contacts buffer).
    shadow_arb->stamp = arb->stamp + spaces.first->stamp - spaces.second->stamp;


    shadow_arb->count = arb->count;
    shadow_arb->n = arb->n;
    shadow_arb->state = arb->state;
    shadow_arb->e = arb->e;
    shadow_arb->u = arb->u;
    shadow_arb->surface_vr = arb->surface_vr;
    shadow_arb->swapped = arb->swapped;


    cpSpacePushFreshContactBuffer(spaces.first);
    shadow_arb->contacts = cpContactBufferGetArray(spaces.first);
    memcpy(shadow_arb->contacts, arb->contacts, arb->count*sizeof(struct cpContact));
    cpSpacePushContacts(spaces.first, shadow_arb->count);

    // Iterate over the possible pairs to look for hash value matches.
//    for(int i=0; i<shadow_arb->count; i++){
//        struct cpContact *con = &shadow_arb->contacts[i];
//        if(con->hash)
//            std::cerr << "HASH-------------------" << con->hash;
//    }

    cpCollisionType typeA = shadow_arb->a->type, typeB = shadow_arb->b->type;
    cpCollisionHandler *defaultHandler = &spaces.first->defaultHandler;
    cpCollisionHandler *handler = shadow_arb->handler = cpSpaceLookupHandler(spaces.first, typeA, typeB, defaultHandler);

    // Check if the types match, but don't swap for a default handler which use the wildcard for type A.
    cpBool swapped = shadow_arb->swapped = (typeA != handler->typeA && handler->typeA != CP_WILDCARD_COLLISION_TYPE);

    if(handler != defaultHandler || spaces.first->usesWildcards){
        // The order of the main handler swaps the wildcard handlers too. Uffda.
        shadow_arb->handlerA = cpSpaceLookupHandler(spaces.first, (swapped ? typeB : typeA), CP_WILDCARD_COLLISION_TYPE, &cpCollisionHandlerDoNothing);
        shadow_arb->handlerB = cpSpaceLookupHandler(spaces.first, (swapped ? typeA : typeB), CP_WILDCARD_COLLISION_TYPE, &cpCollisionHandlerDoNothing);
    }

    // Narrow-phase collision detection.
//    struct cpCollisionInfo info = cpCollide(a, b, id, cpContactBufferGetArray(space));  // struct cpCollisionInfo info = {a, b, id, cpvzero, 0, contacts};
//
//    cpSpacePushContacts(spaces.first, shadow_arb->count);


//    cpArbiterUpdate(arb, &info, space);

    // Time stamp the arbiter so we know it was used recently.
}

void Simulation::reset() {
    cars[0]->reset();
    cars[1]->reset();
    deadline->reset();

    space->stamp+=5;

    // Reset and empty the arbiter lists.
    for(int i=0; i<space->arbiters->num; i++){
        cpArbiter *arb = (cpArbiter *)space->arbiters->arr[i];
        arb->state = CP_ARBITER_STATE_NORMAL;

        // If both bodies are awake, unthread the arbiter from the contact graph.
        if(!cpBodyIsSleeping(arb->body_a) && !cpBodyIsSleeping(arb->body_b)){
            cpArbiterUnthread(arb);
        }
    }
    space->arbiters->num = 0;

    cpHashSetFilter(space->cachedArbiters, (cpHashSetFilterFunc)cachedArbitersRemoveAll, space);

//    std::pair<cpSpace *, cpSpace*> spaces = std::make_pair(space, linked_space);
//    cpHashSetEach(linked_space->cachedArbiters, ( cpHashSetIteratorFunc)copyCachedArbiter, &spaces);
}