#ifndef MINI_AI_CUP_3_UTILS_HPP
#define MINI_AI_CUP_3_UTILS_HPP

#include <chipmunk/chipmunk.h>
#include <chipmunk/chipmunk_structs.h>
#include "../../nlohmann/json.hpp"

struct enemyPrediction {
    cpVect car_pos, rear_wheel_pos, front_wheel_pos;
};

static inline void copy_damp_state(cpDampedSpring *to, cpDampedSpring *from){
    to->jAcc = from->jAcc;
    to->target_vrn = from->target_vrn;
    to->v_coef = from->v_coef;
}

static inline void copy_motor_state(cpSimpleMotor *to, cpSimpleMotor *from){
    to->jAcc = from->jAcc;
}

static inline void copy_groove_state(cpGrooveJoint *to, cpGrooveJoint *from){
    to->jAcc = from->jAcc;
    to->bias = from->bias;
}

static inline void copy_body_state(cpBody *to, cpBody *from) {
    to->p = from->p;
    to->a = from->a;
    to->v = from->v;
    to->w = from->w;
//    to->f = from->f;
//    to->t = from->t;
    to->v_bias = from->v_bias;
    to->w_bias = from->w_bias;
    to->transform = from->transform;
}

static inline void copy_body_position(cpBody *to, cpBody *from) {
    to->p = from->p;
    to->a = from->a;
//    to->v      = from->v;
//    to->w      = from->w;
//    to->v_bias = from->v_bias;
//    to->w_bias = from->w_bias;
    to->transform = from->transform;
}

static inline double err(cpVect pos, cpFloat x, cpFloat y) {
    pos.x -= x;
    pos.y -= y;
    pos.x *= pos.x;
    pos.y *= pos.y;
    return pos.x + pos.y; // squared distance between predicted and real values
}

static inline double get_predict_error(enemyPrediction &prediction, const nlohmann::json &car_params) {
    return err(prediction.car_pos, car_params[0][0].get<cpFloat>(), car_params[0][1].get<cpFloat>()) +
           err(prediction.rear_wheel_pos, car_params[3][0].get<cpFloat>(), car_params[3][1].get<cpFloat>()) +
           err(prediction.front_wheel_pos, car_params[4][0].get<cpFloat>(), car_params[4][1].get<cpFloat>());
}

#ifdef REWIND_VIEWER

#include "../RewindClient.h"

static inline void draw_poly(RewindClient &rw_client, cpBody *body, cpShape *poly, uint32_t color) {
    int count = cpPolyShapeGetCount(poly);
    const cpVect &first = cpBodyLocalToWorld(body, cpPolyShapeGetVert(poly, 0));
    cpVect prev = first;

    for (int i = 1; i < count; i++) {
        const cpVect &curr = cpBodyLocalToWorld(body, cpPolyShapeGetVert(poly, i));
        rw_client.line(prev.x, prev.y, curr.x, curr.y, color);
        prev = curr;
    }
    rw_client.line(prev.x, prev.y, first.x, first.y, color);
}

#endif

#endif //MINI_AI_CUP_3_UTILS_HPP
