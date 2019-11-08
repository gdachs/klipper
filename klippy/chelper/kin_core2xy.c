// Core2XY kinematics stepper pulse time generation
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2019  Gerald Dachs <gda@dachsweb.de>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

struct dual_carriage {
    struct stepper_kinematics sk;
    double offset;
};

static double
core2xy_stepper_plus_calc_position(struct stepper_kinematics *sk, struct move *m
                                  , double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    return c.x + c.y;
}

static double
core2xy_stepper_minus_calc_position(struct stepper_kinematics *sk, struct move *m
                                   , double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    return c.x - c.y;
}

static double
core2xy_stepper_dc_park_calc_position(struct stepper_kinematics *sk, struct move *m
                                  , double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    return c.y;
}

static double
core2xy_stepper_dc_copy_calc_position(struct stepper_kinematics *sk, struct move *m
                                  , double move_time)
{
    struct dual_carriage *dc = container_of(sk, struct dual_carriage, sk);
    struct coord c = move_get_coord(m, move_time);
    return c.x + c.y + dc->offset;
}

static double
core2xy_stepper_dc_mirror_calc_position(struct stepper_kinematics *sk, struct move *m
                                  , double move_time)
{
    struct dual_carriage *dc = container_of(sk, struct dual_carriage, sk);
    struct coord c = move_get_coord(m, move_time);
    return -c.x + c.y + dc->offset;
}

struct stepper_kinematics * __visible
core2xy_stepper_alloc(char type, double offset)
{
    struct dual_carriage *dc = malloc(sizeof(*dc));
    memset(dc, 0, sizeof(*dc));
    dc->offset = offset;
    if (type == '+')
        dc->sk.calc_position_cb = core2xy_stepper_plus_calc_position;
    else if (type == '-')
        dc->sk.calc_position_cb = core2xy_stepper_minus_calc_position;
    else if (type == 'P')
        dc->sk.calc_position_cb = core2xy_stepper_dc_park_calc_position;
    else if (type == 'C')
        dc->sk.calc_position_cb = core2xy_stepper_dc_copy_calc_position;
    else if (type == 'M')
        dc->sk.calc_position_cb = core2xy_stepper_dc_mirror_calc_position;
    return &dc->sk;
}
