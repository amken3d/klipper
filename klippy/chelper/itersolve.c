#include <math.h>   // sqrt, fabs
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h"    // __visible
#include "itersolve.h"   // struct coord
#include "pyhelper.h"    // errorf
#include "stepcompress.h" // queue_append_start

/****************************************************************
 * Kinematic Moves and Ramps
 ****************************************************************/

struct move * __visible move_alloc(void) {
    struct move *m = malloc(sizeof(*m));
    memset(m, 0, sizeof(*m));
    return m;
}

// SixPoint™ ramp generation setup for TMC5160
void __visible move_fill_sixpoint(struct move *m, double print_time,
                                  double a1, double v1, double amax, double dmax,
                                  double start_pos_x, double start_pos_y, double start_pos_z,
                                  double axes_d_x, double axes_d_y, double axes_d_z,
                                  double start_v, double cruise_v, double accel) {
    m->print_time = print_time;
    m->move_t = a1 + v1 + amax + dmax;
    m->accel_t = a1;
    m->cruise_t = v1;
    m->cruise_start_d = a1 * .5 * (cruise_v + start_v);
    m->decel_start_d = m->cruise_start_d + v1 * cruise_v;

    // Setup acceleration and deceleration for SixPoint™ ramp
    m->cruise_v = cruise_v;
    m->accel.c1 = start_v;
    m->accel.c2 = .5 * accel;
    m->decel.c1 = cruise_v;
    m->decel.c2 = -m->accel.c2;

    // Setup for move_get_coord()
    m->start_pos.x = start_pos_x;
    m->start_pos.y = start_pos_y;
    m->start_pos.z = start_pos_z;
    double inv_move_d = 1. / sqrt(axes_d_x * axes_d_x + axes_d_y * axes_d_y + axes_d_z * axes_d_z);
    m->axes_r.x = axes_d_x * inv_move_d;
    m->axes_r.y = axes_d_y * inv_move_d;
    m->axes_r.z = axes_d_z * inv_move_d;
}

/****************************************************************
 * Iterative Solver for TMC5160's SixPoint™ ramp
 ****************************************************************/

struct timepos {
    double time, position;
};

// Find step using false position method
static struct timepos itersolve_find_step(struct stepper_kinematics *sk, struct move *m,
                                          struct timepos low, struct timepos high, double target) {
    sk_callback calc_position = sk->calc_position;
    struct timepos best_guess = high;
    low.position -= target;
    high.position -= target;
    if (!high.position)
        return best_guess;
    int high_sign = signbit(high.position);
    if (high_sign == signbit(low.position))
        return (struct timepos){low.time, target};
    for (;;) {
        double guess_time = ((low.time * high.position - high.time * low.position)
                             / (high.position - low.position));
        if (fabs(guess_time - best_guess.time) <= .000000001)
            break;
        best_guess.time = guess_time;
        best_guess.position = calc_position(sk, m, guess_time);
        double guess_position = best_guess.position - target;
        int guess_sign = signbit(guess_position);
        if (guess_sign == high_sign) {
            high.time = guess_time;
            high.position = guess_position;
        } else {
            low.time = guess_time;
            low.position = guess_position;
        }
    }
    return best_guess;
}

// Generate step times for TMC5160's SixPoint™ ramp during a move
int32_t __visible itersolve_gen_steps(struct stepper_kinematics *sk, struct move *m) {
    struct stepcompress *sc = sk->sc;
    sk_callback calc_position = sk->calc_position;
    double half_step = .5 * sk->step_dist;
    double mcu_freq = stepcompress_get_mcu_freq(sc);
    struct timepos last = {0., sk->commanded_pos}, low = last, high = last;
    double seek_time_delta = 0.000100;
    int sdir = stepcompress_get_step_dir(sc);
    struct queue_append qa = queue_append_start(sc, m->print_time, .5);

    for (;;) {
        double dist = high.position - last.position;
        if (fabs(dist) < half_step) {
        seek_new_high_range:
            if (high.time >= m->move_t)
                break;
            low = high;
            high.time = last.time + seek_time_delta;
            seek_time_delta += seek_time_delta;
            if (high.time > m->move_t)
                high.time = m->move_t;
            high.position = calc_position(sk, m, high.time);
            continue;
        }
        int next_sdir = dist > 0.;
        if (unlikely(next_sdir != sdir)) {
            if (fabs(dist) < half_step + .000000001)
                goto seek_new_high_range;
            if (last.time >= low.time && high.time > last.time) {
                high.time = (last.time + high.time) * .5;
                high.position = calc_position(sk, m, high.time);
                continue;
            }
            int ret = queue_append_set_next_step_dir(&qa, next_sdir);
            if (ret)
                return ret;
            sdir = next_sdir;
        }
        double target = last.position + (sdir ? half_step : -half_step);
        struct timepos next = itersolve_find_step(sk, m, low, high, target);
        int ret = queue_append(&qa, next.time * mcu_freq);
        if (ret)
            return ret;
        seek_time_delta = next.time - last.time;
        if (seek_time_delta < .000000001)
            seek_time_delta = .000000001;
        last.position = target + (sdir ? half_step : -half_step);
        last.time = next.time;
        low = next;
        if (last.time >= high.time)
            goto seek_new_high_range;
    }
    queue_append_finish(qa);
    sk->commanded_pos = last.position;
    return 0;
}

void __visible itersolve_set_stepcompress(struct stepper_kinematics *sk, struct stepcompress *sc,
                                          double step_dist, bool spi_motion) {
    sk->sc = sc;
    sk->step_dist = step_dist;
    sk->spi_motion = spi_motion;
}

double __visible itersolve_calc_position_from_coord(struct stepper_kinematics *sk, double x, double y, double z) {
    struct move m;
    memset(&m, 0, sizeof(m));
    move_fill_sixpoint(&m, 0., 0., 1., 0., x, y, z, 0., 1., 0., 0., 1., 0.);
    return sk->calc_position(sk, &m, 0.);
}

void __visible itersolve_set_commanded_pos(struct stepper_kinematics *sk, double pos) {
    sk->commanded_pos = pos;
}

double __visible itersolve_get_commanded_pos(struct stepper_kinematics *sk) {
    return sk->commanded_pos;
}
