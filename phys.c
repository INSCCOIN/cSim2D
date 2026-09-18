
#include "sim.h"
#include <math.h>
#include <string.h>

void sim_init(Sim *s)
{
    memset(s, 0, sizeof *s);
    s->gy = -22.f;
    s->camy = 4.5f;
    s->camz = -8.f;
    s->foc = 210.f;
}

int sim_add_sphere(Sim *s, float x, float y, float z, float r, float m)
{
    SimBody *b;
    if (s->n >= SIM_MAX) return -1;
    b = &s->b[s->n];
    memset(b, 0, sizeof *b);
    b->x = x; b->y = y; b->z = z; b->r = r;
    b->hx = b->hy = b->hz = r;
    b->m = m < 0.02f ? 0.02f : m;
    b->im = 1.f / b->m;
    b->shape = SIM_SPHERE;
    b->col = s->n * 41;
    return s->n++;
}

int sim_add_box(Sim *s, float x, float y, float z, float sx, float sy, float sz, float m)
{
    SimBody *b;
    if (s->n >= SIM_MAX) return -1;
    b = &s->b[s->n];
    memset(b, 0, sizeof *b);
    b->x = x; b->y = y; b->z = z;
    b->hx = sx * 0.5f; b->hy = sy * 0.5f; b->hz = sz * 0.5f;
    b->r = sqrtf(b->hx * b->hx + b->hy * b->hy + b->hz * b->hz);
    b->m = m < 0.02f ? 0.02f : m;
    b->im = 1.f / b->m;
    b->shape = SIM_BOX;
    b->col = s->n * 41;
    return s->n++;
}

void sim_static(Sim *s, int id)
{
    if (id < 0 || id >= s->n) return;
    s->b[id].static_ = 1;
    s->b[id].im = 0;
    s->b[id].m = 0;
}

void sim_kick(Sim *s, int id, float fx, float fy, float fz)
{
    if (id < 0 || id >= s->n || s->b[id].static_) return;
    s->b[id].vx += fx * s->b[id].im;
    s->b[id].vy += fy * s->b[id].im;
    s->b[id].vz += fz * s->b[id].im;
}

void sim_cam_follow(Sim *s, int id)
{
    float tx, tz;
    if (id < 0 || id >= s->n) return;
    tx = s->b[id].x - sinf(s->yaw) * 7.f;
    tz = s->b[id].z - cosf(s->yaw) * 7.f;
    s->camx += (tx - s->camx) * 0.1f;
    s->camz += (tz - s->camz) * 0.1f;
    s->camy += (s->b[id].y + 3.2f - s->camy) * 0.08f;
}

static void extents(const SimBody *b, float *x0, float *y0, float *z0, float *x1, float *y1, float *z1)
{
    *x0 = b->x - b->hx; *y0 = b->y - b->hy; *z0 = b->z - b->hz;
    *x1 = b->x + b->hx; *y1 = b->y + b->hy; *z1 = b->z + b->hz;
}

static void resolve(SimBody *a, SimBody *b, float nx, float ny, float nz, float pen)
{
    float rv, j, im, e = 0.18f, mu = 0.45f, tx, ty, tz, tl, vt, jt;
    if (pen <= 0.f) return;
    im = a->im + b->im;
    if (im <= 0.f) return;
    a->x -= nx * pen * (a->im / im);
    a->y -= ny * pen * (a->im / im);
    a->z -= nz * pen * (a->im / im);
    b->x += nx * pen * (b->im / im);
    b->y += ny * pen * (b->im / im);
    b->z += nz * pen * (b->im / im);
    rv = (b->vx - a->vx) * nx + (b->vy - a->vy) * ny + (b->vz - a->vz) * nz;
    if (rv > 0.f) return;
    j = -(1.f + e) * rv / im;
    a->vx -= a->im * j * nx; a->vy -= a->im * j * ny; a->vz -= a->im * j * nz;
    b->vx += b->im * j * nx; b->vy += b->im * j * ny; b->vz += b->im * j * nz;
    tx = (b->vx - a->vx) - rv * nx;
    ty = (b->vy - a->vy) - rv * ny;
    tz = (b->vz - a->vz) - rv * nz;
    tl = sqrtf(tx * tx + ty * ty + tz * tz);
    if (tl < 1e-5f) return;
    tx /= tl; ty /= tl; tz /= tl;
    vt = (b->vx - a->vx) * tx + (b->vy - a->vy) * ty + (b->vz - a->vz) * tz;
    jt = -vt / im;
    if (jt > j * mu) jt = j * mu;
    if (jt < -j * mu) jt = -j * mu;
    a->vx -= a->im * jt * tx; a->vy -= a->im * jt * ty; a->vz -= a->im * jt * tz;
    b->vx += b->im * jt * tx; b->vy += b->im * jt * ty; b->vz += b->im * jt * tz;
}

static void pair(SimBody *a, SimBody *b)
{
    float ax0,ay0,az0,ax1,ay1,az1,bx0,by0,bz0,bx1,by1,bz1,ox,oy,oz,nx=1,ny=0,nz=0,pen;
    if (a->shape == SIM_SPHERE && b->shape == SIM_SPHERE) {
        float dx = b->x - a->x, dy = b->y - a->y, dz = b->z - a->z;
        float d = sqrtf(dx*dx+dy*dy+dz*dz), r = a->r + b->r;
        if (d < 1e-5f || d >= r) return;
        resolve(a, b, dx/d, dy/d, dz/d, r-d);
        return;
    }
    extents(a,&ax0,&ay0,&az0,&ax1,&ay1,&az1);
    extents(b,&bx0,&by0,&bz0,&bx1,&by1,&bz1);
    if (ax1<bx0||bx1<ax0||ay1<by0||by1<ay0||az1<bz0||bz1<az0) return;
    ox = fminf(ax1,bx1)-fmaxf(ax0,bx0);
    oy = fminf(ay1,by1)-fmaxf(ay0,by0);
    oz = fminf(az1,bz1)-fmaxf(az0,bz0);
    pen = ox; nx = (a->x < b->x)?1.f:-1.f; ny = 0; nz = 0;
    if (oy < pen) { pen = oy; nx = 0; ny = (a->y < b->y)?1.f:-1.f; nz = 0; }
    if (oz < pen) { pen = oz; nx = 0; ny = 0; nz = (a->z < b->z)?1.f:-1.f; }
    resolve(a,b,nx,ny,nz,pen);
}

static void floor_hit(SimBody *b)
{
    float pen;
    if (b->static_) return;
    pen = 0.f - (b->y - b->hy);
    if (pen <= 0.f) return;
    b->y += pen;
    if (b->vy < 0.f) b->vy = -b->vy * 0.22f;
    b->vx *= 0.82f; b->vz *= 0.82f;
    if (fabsf(b->vy) < 0.4f) b->vy = 0;
}

void sim_step(Sim *s, float dt)
{
    int i,j,k;
    if (dt > 0.025f) dt = 0.025f;
    for (k = 0; k < 3; k++) {
        float h = dt / 3.f;
        for (i = 0; i < s->n; i++) {
            SimBody *b = &s->b[i];
            if (b->static_) continue;
            b->vy += s->gy * h;
            b->vx *= 0.9992f; b->vz *= 0.9992f;
            b->x += b->vx * h; b->y += b->vy * h; b->z += b->vz * h;
            floor_hit(b);
        }
        for (i = 0; i < s->n; i++)
            for (j = i+1; j < s->n; j++)
                pair(&s->b[i], &s->b[j]);
        for (i = 0; i < s->n; i++) floor_hit(&s->b[i]);
    }
}
