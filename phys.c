#include "sim.h"
#include <math.h>
#include <string.h>

#define CELL 2.0f
#define GDIM 32
#define BUCK 8

void sim_init(Sim *s)
{
    memset(s, 0, sizeof *s);
    s->gy = -22.f;
    s->ge = 0.28f;
    s->gmu = 0.55f;
    s->camy = 4.5f;
    s->camz = -8.f;
    s->foc = 210.f;
}

static void wake(SimBody *b)
{
    b->sleep = 0;
    b->awake = 1;
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
    b->I = 0.4f * b->m * r * r;
    b->iI = b->I > 0 ? 1.f / b->I : 0;
    b->e = 0.35f; b->mu = 0.4f;
    b->shape = SIM_SPHERE; b->awake = 1;
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
    b->I = b->m * (b->hx * b->hx + b->hz * b->hz) / 6.f;
    b->iI = b->I > 0 ? 1.f / b->I : 0;
    b->e = 0.18f; b->mu = 0.5f;
    b->shape = SIM_BOX; b->awake = 1;
    b->col = s->n * 41;
    return s->n++;
}

void sim_static(Sim *s, int id)
{
    if (id < 0 || id >= s->n) return;
    s->b[id].static_ = 1;
    s->b[id].im = 0; s->b[id].iI = 0; s->b[id].m = 0;
    s->b[id].awake = 0;
}

void sim_kick(Sim *s, int id, float fx, float fy, float fz)
{
    if (id < 0 || id >= s->n || s->b[id].static_) return;
    wake(&s->b[id]);
    s->b[id].vx += fx * s->b[id].im;
    s->b[id].vy += fy * s->b[id].im;
    s->b[id].vz += fz * s->b[id].im;
}

void sim_cam_follow(Sim *s, int id)
{
    float tx, tz;
    int i;
    if (id < 0 || id >= s->n) return;
    tx = s->b[id].x - sinf(s->yaw) * 7.f;
    tz = s->b[id].z - cosf(s->yaw) * 7.f;
    s->camx += (tx - s->camx) * 0.1f;
    s->camz += (tz - s->camz) * 0.1f;
    s->camy += (s->b[id].y + 3.2f - s->camy) * 0.08f;
    for (i = 0; i < s->n; i++) {
        float dx = s->camx - s->b[i].x, dz = s->camz - s->b[i].z;
        float d = sqrtf(dx * dx + dz * dz);
        float need = s->b[i].r + 0.55f;
        if (d < 1e-3f) continue;
        if (d < need && fabsf(s->camy - s->b[i].y) < s->b[i].hy + 1.6f) {
            float p = (need - d) / d;
            s->camx += dx * p;
            s->camz += dz * p;
        }
    }
}

static void rot_xz(float yaw, float x, float z, float *ox, float *oz)
{
    float c = cosf(yaw), si = sinf(yaw);
    *ox = x * c - z * si;
    *oz = x * si + z * c;
}

static void local_xz(float yaw, float x, float z, float *ox, float *oz)
{
    float c = cosf(yaw), si = sinf(yaw);
    *ox = x * c + z * si;
    *oz = -x * si + z * c;
}

int sim_joint_dist(Sim *s, int a, int b, float rest)
{
    SimJoint *j;
    if (s->nj >= SIM_JMAX || a < 0 || b < 0) return -1;
    j = &s->j[s->nj];
    memset(j, 0, sizeof *j);
    j->type = J_DIST; j->a = a; j->b = b; j->rest = rest;
    return s->nj++;
}

int sim_joint_hinge(Sim *s, int a, int b, float wx, float wy, float wz)
{
    SimJoint *j;
    float lx, lz;
    if (s->nj >= SIM_JMAX) return -1;
    j = &s->j[s->nj];
    memset(j, 0, sizeof *j);
    j->type = J_HINGE; j->a = a; j->b = b; j->ay = wy;
    local_xz(s->b[a].yaw, wx - s->b[a].x, wz - s->b[a].z, &lx, &lz);
    j->lax = lx; j->laz = lz;
    local_xz(s->b[b].yaw, wx - s->b[b].x, wz - s->b[b].z, &lx, &lz);
    j->lbx = lx; j->lbz = lz;
    return s->nj++;
}

int sim_joint_slide(Sim *s, int a, int b, float ax, float ay, float az)
{
    SimJoint *j;
    float L;
    if (s->nj >= SIM_JMAX) return -1;
    j = &s->j[s->nj];
    memset(j, 0, sizeof *j);
    j->type = J_SLIDE; j->a = a; j->b = b;
    L = sqrtf(ax * ax + ay * ay + az * az);
    if (L < 1e-4f) L = 1;
    j->ax = ax / L; j->ay = ay / L; j->az = az / L;
    return s->nj++;
}

static void resolve(SimBody *a, SimBody *b, float nx, float ny, float nz,
                    float px, float py, float pz, float pen)
{
    float rv, j, im, e, mu, tx, ty, tz, tl, vt, jt;
    float rax, raz, rbx, rbz, wa, wb;
    if (pen <= 0.f) return;
    if (!a->awake && !a->static_) wake(a);
    if (!b->awake && !b->static_) wake(b);
    im = a->im + b->im;
    if (im <= 0.f) return;
    /* slop so contacts don't weld */
    {
        float corr = pen - 0.012f;
        if (corr < 0.f)
            corr = 0.f;
        if (corr > 0.18f)
            corr = 0.18f;
        a->x -= nx * corr * (a->im / im);
        a->y -= ny * corr * (a->im / im);
        a->z -= nz * corr * (a->im / im);
        b->x += nx * corr * (b->im / im);
        b->y += ny * corr * (b->im / im);
        b->z += nz * corr * (b->im / im);
    }
    /* yaw lever in XZ */
    rax = px - a->x; raz = pz - a->z;
    rbx = px - b->x; rbz = pz - b->z;
    wa = a->wy * (-raz * nx + rax * nz);
    wb = b->wy * (-rbz * nx + rbx * nz);
    rv = (b->vx - a->vx) * nx + (b->vy - a->vy) * ny + (b->vz - a->vz) * nz + wb - wa;
    if (rv > 0.f) return;
    e = fminf(a->e, b->e);
    mu = fminf(a->mu, b->mu) * 0.55f;
    j = -(1.f + e) * rv / (im + 0.0001f);
    a->vx -= a->im * j * nx; a->vy -= a->im * j * ny; a->vz -= a->im * j * nz;
    b->vx += b->im * j * nx; b->vy += b->im * j * ny; b->vz += b->im * j * nz;
    a->wy -= a->iI * j * (-raz * nx + rax * nz);
    b->wy += b->iI * j * (-rbz * nx + rbx * nz);
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

static int sphere_box(SimBody *sp, SimBody *bx)
{
    float lx, lz, ly, cx, cz, cy, wx, wz, dx, dy, dz, d, pen;
    float nx, ny, nz, px, py, pz;
    local_xz(bx->yaw, sp->x - bx->x, sp->z - bx->z, &lx, &lz);
    ly = sp->y - bx->y;
    /* center inside the box: push out on the shallowest face */
    if (fabsf(lx) <= bx->hx && fabsf(lz) <= bx->hz && fabsf(ly) <= bx->hy) {
        float ex = bx->hx - fabsf(lx);
        float ey = bx->hy - fabsf(ly);
        float ez = bx->hz - fabsf(lz);
        if (ex <= ey && ex <= ez) {
            nx = (lx >= 0.f) ? 1.f : -1.f;
            ny = 0;
            nz = 0;
            pen = ex + sp->r;
        } else if (ey <= ez) {
            nx = 0;
            ny = (ly >= 0.f) ? 1.f : -1.f;
            nz = 0;
            pen = ey + sp->r;
        } else {
            nx = 0;
            ny = 0;
            nz = (lz >= 0.f) ? 1.f : -1.f;
            pen = ez + sp->r;
        }
        /* n is box-local from center toward sphere; world it */
        {
            float wnx, wnz;
            rot_xz(bx->yaw, nx, nz, &wnx, &wnz);
            nx = wnx;
            nz = wnz;
        }
        /* resolve expects n from a(sphere) to b(box) = -outward */
        resolve(sp, bx, -nx, -ny, -nz, sp->x, sp->y, sp->z, pen);
        return 1;
    }
    cx = fmaxf(-bx->hx, fminf(bx->hx, lx));
    cz = fmaxf(-bx->hz, fminf(bx->hz, lz));
    cy = fmaxf(-bx->hy, fminf(bx->hy, ly));
    rot_xz(bx->yaw, cx, cz, &wx, &wz);
    wx += bx->x;
    wz += bx->z;
    py = bx->y + cy;
    dx = sp->x - wx;
    dy = sp->y - py;
    dz = sp->z - wz;
    d = sqrtf(dx * dx + dy * dy + dz * dz);
    if (d < 1e-5f || d >= sp->r)
        return 0;
    pen = sp->r - d;
    /* n from sphere(a) to box(b) points toward the box */
    nx = -dx / d;
    ny = -dy / d;
    nz = -dz / d;
    px = wx;
    pz = wz;
    resolve(sp, bx, nx, ny, nz, px, py, pz, pen);
    return 1;
}

static int box_box(SimBody *a, SimBody *b)
{
    /* Y slab + XZ as circles of inradius if yawed much, else AABB */
    float ay0 = a->y - a->hy, ay1 = a->y + a->hy;
    float by0 = b->y - b->hy, by1 = b->y + b->hy;
    float oy, ox, oz, pen, nx, ny, nz, px, py, pz;
    if (ay1 < by0 || by1 < ay0) return 0;
    oy = fminf(ay1, by1) - fmaxf(ay0, by0);
    {
        float ax0 = a->x - a->hx, ax1 = a->x + a->hx;
        float bx0 = b->x - b->hx, bx1 = b->x + b->hx;
        float az0 = a->z - a->hz, az1 = a->z + a->hz;
        float bz0 = b->z - b->hz, bz1 = b->z + b->hz;
        if (fabsf(a->yaw) > 0.05f || fabsf(b->yaw) > 0.05f) {
            float dx = b->x - a->x, dz = b->z - a->z;
            float d = sqrtf(dx * dx + dz * dz);
            float ra = sqrtf(a->hx * a->hx + a->hz * a->hz) * 0.78f;
            float rb = sqrtf(b->hx * b->hx + b->hz * b->hz) * 0.78f;
            if (d < 1e-4f || d >= ra + rb) {
                if (oy < 0.02f) return 0;
            } else {
                pen = ra + rb - d;
                if (oy < pen) {
                    pen = oy; nx = 0; ny = (a->y < b->y) ? 1.f : -1.f; nz = 0;
                } else {
                    nx = dx / d; ny = 0; nz = dz / d;
                }
                px = (a->x + b->x) * 0.5f; py = (a->y + b->y) * 0.5f; pz = (a->z + b->z) * 0.5f;
                resolve(a, b, nx, ny, nz, px, py, pz, pen);
                return 1;
            }
        }
        if (ax1 < bx0 || bx1 < ax0 || az1 < bz0 || bz1 < az0) return 0;
        ox = fminf(ax1, bx1) - fmaxf(ax0, bx0);
        oz = fminf(az1, bz1) - fmaxf(az0, bz0);
        pen = ox; nx = (a->x < b->x) ? 1.f : -1.f; ny = 0; nz = 0;
        if (oy < pen) { pen = oy; nx = 0; ny = (a->y < b->y) ? 1.f : -1.f; nz = 0; }
        if (oz < pen) { pen = oz; nx = 0; ny = 0; nz = (a->z < b->z) ? 1.f : -1.f; }
        px = (a->x + b->x) * 0.5f; py = (a->y + b->y) * 0.5f; pz = (a->z + b->z) * 0.5f;
        resolve(a, b, nx, ny, nz, px, py, pz, pen);
        return 1;
    }
}

static void pair(SimBody *a, SimBody *b)
{
    if (a->static_ && b->static_) return;
    if (!a->awake && !b->awake && !a->static_ && !b->static_) return;
    if (a->shape == SIM_SPHERE && b->shape == SIM_SPHERE) {
        float dx = b->x - a->x, dy = b->y - a->y, dz = b->z - a->z;
        float d = sqrtf(dx * dx + dy * dy + dz * dz), r = a->r + b->r;
        if (d < 1e-5f || d >= r) return;
        resolve(a, b, dx / d, dy / d, dz / d, (a->x + b->x) * 0.5f, (a->y + b->y) * 0.5f,
                (a->z + b->z) * 0.5f, r - d);
        return;
    }
    if (a->shape == SIM_SPHERE && b->shape == SIM_BOX) { sphere_box(a, b); return; }
    if (a->shape == SIM_BOX && b->shape == SIM_SPHERE) { sphere_box(b, a); return; }
    box_box(a, b);
}

static void floor_hit(Sim *s, SimBody *b)
{
    float pen;
    if (b->static_) return;
    pen = 0.f - (b->y - b->hy);
    if (pen <= 0.f) return;
    wake(b);
    b->y += pen;
    if (b->vy < 0.f) b->vy = -b->vy * s->ge;
    b->vx *= (1.f - s->gmu * 0.35f);
    b->vz *= (1.f - s->gmu * 0.35f);
    b->wy *= (1.f - s->gmu * 0.2f);
    if (fabsf(b->vy) < 0.35f) b->vy = 0;
}

static void joints(Sim *s)
{
    int i;
    for (i = 0; i < s->nj; i++) {
        SimJoint *j = &s->j[i];
        SimBody *a = &s->b[j->a], *b = &s->b[j->b];
        float px, pz, qx, qz, dx, dy, dz, d, n, im, corr;
        wake(a); wake(b);
        if (j->type == J_DIST || j->type == J_HINGE) {
            if (j->type == J_HINGE) {
                rot_xz(a->yaw, j->lax, j->laz, &px, &pz); px += a->x; pz += a->z;
                rot_xz(b->yaw, j->lbx, j->lbz, &qx, &qz); qx += b->x; qz += b->z;
                dx = qx - px; dy = (b->y - a->y); dz = qz - pz;
                d = sqrtf(dx * dx + dy * dy + dz * dz);
                n = 0.02f;
            } else {
                dx = b->x - a->x; dy = b->y - a->y; dz = b->z - a->z;
                d = sqrtf(dx * dx + dy * dy + dz * dz);
                n = j->rest;
            }
            if (d < 1e-4f) continue;
            corr = d - n;
            im = a->im + b->im;
            if (im <= 0) continue;
            dx /= d; dy /= d; dz /= d;
            a->x += dx * corr * (a->im / im);
            a->y += dy * corr * (a->im / im);
            a->z += dz * corr * (a->im / im);
            b->x -= dx * corr * (b->im / im);
            b->y -= dy * corr * (b->im / im);
            b->z -= dz * corr * (b->im / im);
        } else {
            float rx = b->x - a->x, ry = b->y - a->y, rz = b->z - a->z;
            float along = rx * j->ax + ry * j->ay + rz * j->az;
            float px2 = rx - along * j->ax, py2 = ry - along * j->ay, pz2 = rz - along * j->az;
            im = a->im + b->im;
            if (im <= 0) continue;
            a->x += px2 * (a->im / im);
            a->y += py2 * (a->im / im);
            a->z += pz2 * (a->im / im);
            b->x -= px2 * (b->im / im);
            b->y -= py2 * (b->im / im);
            b->z -= pz2 * (b->im / im);
        }
    }
}

void sim_step(Sim *s, float dt)
{
    int i, k;
    int buck[GDIM][GDIM][BUCK], nb[GDIM][GDIM];
    if (dt > 0.025f) dt = 0.025f;
    memset(nb, 0, sizeof nb);
    for (k = 0; k < 3; k++) {
        float h = dt / 3.f;
        for (i = 0; i < s->n; i++) {
            SimBody *b = &s->b[i];
            float spd;
            if (b->static_ || !b->awake) continue;
            b->vy += s->gy * h;
            b->vx *= 0.9993f; b->vz *= 0.9993f; b->wy *= 0.997f;
            b->x += b->vx * h; b->y += b->vy * h; b->z += b->vz * h;
            b->yaw += b->wy * h;
            floor_hit(s, b);
            spd = fabsf(b->vx) + fabsf(b->vy) + fabsf(b->vz) + fabsf(b->wy);
            if (spd < 0.05f) {
                if (++b->sleep > 20) {
                    b->awake = 0; b->vx = b->vy = b->vz = b->wy = 0;
                }
            } else b->sleep = 0;
        }
        memset(nb, 0, sizeof nb);
        for (i = 0; i < s->n; i++) {
            int cx = (int)floorf(s->b[i].x / CELL) + GDIM / 2;
            int cz = (int)floorf(s->b[i].z / CELL) + GDIM / 2;
            int u, v;
            if (cx < 1) cx = 1; if (cx > GDIM - 2) cx = GDIM - 2;
            if (cz < 1) cz = 1; if (cz > GDIM - 2) cz = GDIM - 2;
            for (u = cx - 1; u <= cx + 1; u++)
                for (v = cz - 1; v <= cz + 1; v++) {
                    int t, nn = nb[u][v];
                    for (t = 0; t < nn; t++)
                        if (buck[u][v][t] < i)
                            pair(&s->b[buck[u][v][t]], &s->b[i]);
                }
            if (nb[cx][cz] < BUCK)
                buck[cx][cz][nb[cx][cz]++] = i;
        }
        joints(s);
        for (i = 0; i < s->n; i++) floor_hit(s, &s->b[i]);
    }
}
