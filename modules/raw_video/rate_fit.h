#ifndef _rate_fit_h_
#define _rate_fit_h_

/* Least-squares rate of periodic events (video frames, audio chunks): a
 * line fitted through all of a take's (position, time) pairs, so one late
 * or early event barely moves it (unlike first-to-last spans or a running
 * average).  position: frame index or sample count; time in microseconds,
 * from the same clock for all users (get_us_clock), so rates measured this
 * way can be compared with each other. */

#include <stdint.h>
#include <string.h>

struct rate_fit
{
    uint32_t n;
    uint64_t x0, t0;            /* first event, subtracted: keeps the sums small */
    double sx, st, sxx, sxt, stt;
};

static inline void rate_fit_reset(struct rate_fit *f)
{
    memset(f, 0, sizeof(*f));
}

static inline void rate_fit_add(struct rate_fit *f, uint64_t x, uint64_t t)
{
    if (!f->n)
    {
        f->x0 = x;
        f->t0 = t;
    }
    double dx = (double) (x - f->x0);
    double dt = (double) (t - f->t0);
    f->n++;
    f->sx += dx;
    f->st += dt;
    f->sxx += dx * dx;
    f->sxt += dx * dt;
    f->stt += dt * dt;
}

/* microseconds per position unit; 0 with fewer than 8 events */
static inline double rate_fit_period_us(const struct rate_fit *f)
{
    if (f->n < 8)
    {
        return 0;
    }
    double num = (double) f->n * f->sxt - f->sx * f->st;
    double den = (double) f->n * f->sxx - f->sx * f->sx;
    return (num > 0 && den > 0) ? num / den : 0;
}

/* the fitted rate's standard error in ppm (from the scatter of the events
 * around the line); 0 if unknown */
static inline int rate_fit_err_ppm(const struct rate_fit *f)
{
    double p = rate_fit_period_us(f);
    if (p <= 0 || f->n < 8)
    {
        return 0;
    }
    double n = f->n;
    double sxx = f->sxx - f->sx * f->sx / n;
    double stt = f->stt - f->st * f->st / n;
    double sxt = f->sxt - f->sx * f->st / n;
    double sse = stt - p * sxt;
    if (sse <= 0 || sxx <= 0)
    {
        return 0;
    }
    /* standard error of the slope, relative: sqrt(sse / (n - 2) / sxx) / p;
     * no libm here: Newton's square root */
    double v = sse / (n - 2) / sxx;
    double r = v > 1 ? v : 1;
    for (int i = 0; i < 40; i++)
    {
        r = (r + v / r) / 2;
    }
    return (int) (r / p * (double) 1000000 + (double) 0.5);
}

/* position units per second (fps, Hz); 0 if unknown */
static inline double rate_fit_rate(const struct rate_fit *f)
{
    double p = rate_fit_period_us(f);
    return p > 0 ? (double) 1000000 / p : 0;
}

#endif
