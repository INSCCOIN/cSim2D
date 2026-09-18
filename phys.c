#include "sim.h"
#include <math.h>
#include <string.h>

void sim_init(Sim *s)
{
    memset(s, 0, sizeof *s);
    s->gy = -18.f;
    s->dt = 1.f / 60.f;
    s->zoom = 28.f;
    s->pitch = 0.55f;
    s->camy = 3.f;
}

int sim_add_circle(Sim *s, float x, float y, float r, float m)
{
    SimBody *b;
    if (s->n >= SIM_MAX)
        return -1;
    b = &s->b[s->n];
    memset(b, 0, sizeof *b);
    b->x = x;
    b->y = y;
    b->r = r;
    b->m = m < 0.01f ? 0.01f : m;
    b->im = 1.f / b->m;
    b->I = 0.5f * b->m * r * r;
    b->iI = b->I > 0 ? 1.f / b->I : 0;
    b->shape = SIM_CIRCLE;
    b->col = s->n * 37;
    return s->n++;
}

int sim_add_box(Sim *s, float x, float y, float w, float h, float m)
{
    SimBody *b;
    if (s->n >= SIM_MAX)
        return -1;
    b = &s->b[s->n];
    memset(b, 0, sizeof *b);
    b->x = x;
    b->y = y;
    b->hw = w * 0.5f;
    b->hh = h * 0.5f;
    b->r = sqrtf(b->hw * b->hw + b->hh * b->hh);
    b->m = m < 0.01f ? 0.01f : m;
    b->im = 1.f / b->m;
    b->I = b->m * (w * w + h * h) / 12.f;
    b->iI = b->I > 0 ? 1.f / b->I : 0;
    b->shape = SIM_BOX;
    b->col = s->n * 37;
    return s->n++;
}

void sim_static(Sim *s, int id)
{
    if (id < 0 || id >= s->n)
        return;
    s->b[id].static_ = 1;
    s->b[id].im = 0;
    s->b[id].iI = 0;
    s->b[id].m = 0;
}

void sim_kick(Sim *s, int id, float fx, float fy)
{
    if (id < 0 || id >= s->n || s->b[id].static_)
        return;
    s->b[id].vx += fx * s->b[id].im;
    s->b[id].vy += fy * s->b[id].im;
}

void sim_cam_follow(Sim *s, int id)
{
    if (id < 0 || id >= s->n)
        return;
    s->camx += (s->b[id].x - s->camx) * 0.08f;
    s->camy += (s->b[id].y + 1.5f - s->camy) * 0.08f;
}

static void aabb(const SimBody *b, float *x0, float *y0, float *x1, float *y1)
{
    if (b->shape == SIM_CIRCLE) {
        *x0 = b->x - b->r;
        *y0 = b->y - b->r;
        *x1 = b->x + b->r;
        *y1 = b->y + b->r;
    } else {
        *x0 = b->x - b->hw;
        *y0 = b->y - b->hh;
        *x1 = b->x + b->hw;
        *y1 = b->y + b->hh;
    }
}

static void resolve(SimBody *a, SimBody *b, float nx, float ny, float pen)
{
    float rvx, rvy, vn, e = 0.25f, j, im;
    if (pen <= 0)
        return;
    im = a->im + b->im;
    if (im <= 0)
        return;
    a->x -= nx * pen * (a->im / im);
    a->y -= ny * pen * (a->im / im);
    b->x += nx * pen * (b->im / im);
    b->y += ny * pen * (b->im / im);
    rvx = b->vx - a->vx;
    rvy = b->vy - a->vy;
    vn = rvx * nx + rvy * ny;
    if (vn > 0)
        return;
    j = -(1.f + e) * vn / im;
    a->vx -= a->im * j * nx;
    a->vy -= a->im * j * ny;
    b->vx += b->im * j * nx;
    b->vy += b->im * j * ny;
    /* coulomb-ish */
    {
        float tx = -ny, ty = nx;
        float vt = rvx * tx + rvy * ty;
        float jt = -vt / im;
        float mu = 0.35f;
        if (jt > j * mu)
            jt = j * mu;
        if (jt < -j * mu)
            jt = -j * mu;
        a->vx -= a->im * jt * tx;
        a->vy -= a->im * jt * ty;
        b->vx += b->im * jt * tx;
        b->vy += b->im * jt * ty;
    }
}

static void pair(SimBody *a, SimBody *b)
{
    float ax0, ay0, ax1, ay1, bx0, by0, bx1, by1;
    float nx, ny, pen;
    if (a->shape == SIM_CIRCLE && b->shape == SIM_CIRCLE) {
        float dx = b->x - a->x, dy = b->y - a->y;
        float d = sqrtf(dx * dx + dy * dy);
        float r = a->r + b->r;
        if (d < 1e-4f || d >= r)
            return;
        nx = dx / d;
        ny = dy / d;
        pen = r - d;
        resolve(a, b, nx, ny, pen);
        return;
    }
    aabb(a, &ax0, &ay0, &ax1, &ay1);
    aabb(b, &bx0, &by0, &bx1, &by1);
    if (ax1 < bx0 || bx1 < ax0 || ay1 < by0 || by1 < ay0)
        return;
    {
        float px = (ax1 < bx1 ? ax1 : bx1) - (ax0 > bx0 ? ax0 : bx0);
        float py = (ay1 < by1 ? ay1 : by1) - (ay0 > by0 ? ay0 : by0);
        if (px < py) {
            nx = (a->x < b->x) ? 1.f : -1.f;
            ny = 0;
            pen = px;
        } else {
            nx = 0;
            ny = (a->y < b->y) ? 1.f : -1.f;
            pen = py;
        }
    }
    resolve(a, b, nx, ny, pen);
}

void sim_step(Sim *s, float dt)
{
    int i, j, k;
    if (dt > 0.03f)
        dt = 0.03f;
    for (k = 0; k < 2; k++) {
        float h = dt * 0.5f;
        for (i = 0; i < s->n; i++) {
            SimBody *b = &s->b[i];
            if (b->static_)
                continue;
            b->vy += s->gy * h;
            b->vx *= 0.999f;
            b->x += b->vx * h;
            b->y += b->vy * h;
            b->w *= 0.998f;
        }
        for (i = 0; i < s->n; i++)
            for (j = i + 1; j < s->n; j++)
                pair(&s->b[i], &s->b[j]);
    }
}
