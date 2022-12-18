// Cable winch stepper kinematics
//
// Copyright (C) 2022  Dmitry Lavrov
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord


struct pythagoras_stepper{
    struct stepper_kinematics sk;
    double x, y;
    double r1, r2;
};

static double square(double x){
    return x*x;
}

static double pythagoras_stepper_calc_position(struct stepper_kinematics *sk, struct move *m
                            , double move_time)
{
    struct pythagoras_stepper *hs = container_of(sk, struct pythagoras_stepper, sk);
    struct coord c = move_get_coord(m, move_time);
    double dx = c.x - hs->x;
    double dy = c.y - hs->y;
    double d2 = dx*dx + dy*dy;
    double free_belt_length=sqrt(d2 - square(hs->r1 + hs->r2));
    double belt_angle = atan2(dy, dx) - atan2(hs->r1 + hs->r2, free_belt_length);
    double belt_length = free_belt_length - belt_angle * hs->r1;
    return belt_length;
}

struct stepper_kinematics * __visible
pythagoras_stepper_alloc(double x, double y, double r1, double r2)
{
    // Don't know why won't load
    struct pythagoras_stepper *hs = malloc(sizeof(*hs));
    memset(hs, 0, sizeof(*hs));
    hs->x = x;
    hs->y = y;
    hs->r1 = r1;
    hs->r2 = r2;
    hs->sk.calc_position_cb = pythagoras_stepper_calc_position;
    hs->sk.active_flags = AF_X | AF_Y;
    return &hs->sk;
}
