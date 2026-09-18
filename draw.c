#include "sim.h"
#include "fb.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static uint16_t mixfog(uint16_t c, float depth)
{
    int r = ((c >> 11) & 31) << 3, g = ((c >> 5) & 63) << 2, b = (c & 31) << 3;
    float t = depth / 28.f;
    if (t > 1.f) t = 1.f;
    if (t < 0) t = 0;
    r = (int)(r * (1.f - t) + 16 * t);
    g = (int)(g * (1.f - t) + 20 * t);
    b = (int)(b * (1.f - t) + 34 * t);
    return rgb565(r, g, b);
}

static int project(const Sim *s, float x, float y, float z, int *sx, int *sy, float *depth)
{
    float dx = x - s->camx, dy = y - s->camy, dz = z - s->camz;
    float cy = cosf(s->yaw), syw = sinf(s->yaw);
    float rx = dx * cy - dz * syw;
    float rz = dx * syw + dz * cy;
    if (rz < 0.35f) return 0;
    *sx = (int)FB_W / 2 + (int)(s->foc * rx / rz);
    *sy = (int)FB_H / 2 - (int)(s->foc * dy / rz);
    *depth = rz;
    return 1;
}

static uint16_t pal(int col, int lit)
{
    int t[][3] = {{190,75,60},{60,130,200},{70,165,85},{210,175,55},
                  {150,95,185},{210,115,45},{100,105,115},{220,220,230}};
    int i = abs(col) % 8;
    int r = t[i][0] * lit / 100, g = t[i][1] * lit / 100, b = t[i][2] * lit / 100;
    if (r > 255) r = 255;
    if (g > 255) g = 255;
    if (b > 255) b = 255;
    return rgb565(r, g, b);
}

static void floor_mode7(const Sim *s)
{
    int y, x, hy = (int)FB_H / 2 + 8;
    float cy = cosf(s->yaw), syw = sinf(s->yaw);
    for (y = hy; y < (int)FB_H; y += 2) {
        float p = (float)(y - hy + 1) / (float)FB_H;
        float dist = s->camy / (p * 1.35f + 0.02f);
        float step = dist / s->foc * 2.f;
        float wx0 = s->camx + syw * dist - cy * step * ((float)FB_W * 0.25f);
        float wz0 = s->camz + cy * dist + syw * step * ((float)FB_W * 0.25f);
        float dx = cy * step, dz = -syw * step;
        for (x = 0; x < (int)FB_W; x += 2) {
            int ix = (int)floorf(wx0), iz = (int)floorf(wz0);
            int c = (ix + iz) & 1;
            int fade = (int)(80.f + 40.f / (1.f + dist * 0.08f));
            uint16_t col = c ? rgb565(fade / 3, fade / 2, fade / 4)
                             : rgb565(fade / 4, fade / 3, fade / 5);
            col = mixfog(col, dist);
            px(x, y, col); px(x + 1, y, col);
            px(x, y + 1, col); px(x + 1, y + 1, col);
            wx0 += dx; wz0 += dz;
        }
    }
}

static void disc(int cx, int cy, int r, uint16_t c)
{
    int y, x;
    if (r < 1) r = 1;
    if (r > 70) r = 70;
    for (y = -r; y <= r; y++) {
        int w = (int)sqrtf((float)(r * r - y * y));
        for (x = -w; x <= w; x++)
            px(cx + x, cy + y, c);
    }
}

static void tri(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t c)
{
    int i;
    /* bubble sort by y */
    if (y0 > y1) { int t=x0; x0=x1; x1=t; t=y0; y0=y1; y1=t; }
    if (y0 > y2) { int t=x0; x0=x2; x2=t; t=y0; y0=y2; y2=t; }
    if (y1 > y2) { int t=x1; x1=x2; x2=t; t=y1; y1=y2; y2=t; }
    if (y2 == y0) return;
    for (i = y0; i <= y2; i++) {
        int xa, xb;
        if (i < y1 && y1 != y0)
            xa = x0 + (x1 - x0) * (i - y0) / (y1 - y0);
        else if (y2 != y1)
            xa = x1 + (x2 - x1) * (i - y1) / (y2 - y1);
        else xa = x1;
        xb = x0 + (x2 - x0) * (i - y0) / (y2 - y0);
        if (xa > xb) { int t = xa; xa = xb; xb = t; }
        hline(xa, i, xb - xa + 1, c);
    }
}

static void quad(int *xs, int *ys, uint16_t c)
{
    tri(xs[0], ys[0], xs[1], ys[1], xs[2], ys[2], c);
    tri(xs[0], ys[0], xs[2], ys[2], xs[3], ys[3], c);
}

static float sunlit(float nx, float ny, float nz)
{
    /* sun: (-0.35, 0.82, -0.45) */
    float d = nx * -0.35f + ny * 0.82f + nz * -0.45f;
    if (d < 0.f)
        d = 0.f;
    return 0.32f + 0.68f * d;
}

static void draw_box(const Sim *s, const SimBody *b, float depth)
{
    float cy = cosf(b->yaw), si = sinf(b->yaw);
    float cp = cosf(b->pitch), sp = sinf(b->pitch);
    float cr = cosf(b->roll), sr = sinf(b->roll);
    float hx = b->hx, hy = b->hy, hz = b->hz;
    int S[8][2], ok[8], i;
    int faces[3][4] = {{4, 5, 6, 7}, {5, 1, 2, 6}, {4, 0, 3, 7}};
    float fn[3][3] = {{0, 1, 0}, {1, 0, 0}, {0, 0, 1}};
    int corner[8][3] = {
        {-1, -1, -1}, {1, -1, -1}, {1, -1, 1}, {-1, -1, 1},
        {-1, 1, -1}, {1, 1, -1}, {1, 1, 1}, {-1, 1, 1}
    };
    for (i = 0; i < 8; i++) {
        float lx = corner[i][0] * hx, ly = corner[i][1] * hy, lz = corner[i][2] * hz;
        float x1, y1, z1, x2, y2, z2;
        /* roll X, pitch Z, yaw Y — small sit-flat angles */
        y1 = ly * cr - lz * sr;
        z1 = ly * sr + lz * cr;
        x1 = lx;
        x2 = x1 * cp + z1 * sp;
        z2 = -x1 * sp + z1 * cp;
        y2 = y1;
        ok[i] = project(s, b->x + x2 * cy - z2 * si, b->y + y2,
                        b->z + x2 * si + z2 * cy, &S[i][0], &S[i][1], &depth);
    }
    for (i = 0; i < 3; i++) {
        int xs[4], ys[4], k, good = 1;
        float wx, wz, lit;
        for (k = 0; k < 4; k++) {
            int id = faces[i][k];
            if (!ok[id])
                good = 0;
            xs[k] = S[id][0];
            ys[k] = S[id][1];
        }
        wx = fn[i][0] * cy - fn[i][2] * si;
        wz = fn[i][0] * si + fn[i][2] * cy;
        lit = sunlit(wx, fn[i][1], wz);
        if (good)
            quad(xs, ys, mixfog(pal(b->col, (int)(lit * 100.f)), depth));
    }
}

void sim_draw(const Sim *s)
{
    int i, j, n = s->n, ord[SIM_MAX];
    float depth[SIM_MAX];
    char buf[64];
    fb_clear(rgb565(8, 10, 22));
    {
        int y, hy = (int)FB_H / 2 + 8;
        for (y = 0; y < hy; y++) {
            float t = (float)y / (float)hy;
            int r = (int)(10 + t * 40), g = (int)(14 + t * 50), b = (int)(32 + t * 55);
            hline(0, y, (int)FB_W, rgb565(r, g, b));
        }
    }
    floor_mode7(s);

    for (i = 0; i < n; i++) {
        int sx, sy; float d = 1e9f;
        ord[i] = i;
        if (!project(s, s->b[i].x, s->b[i].y, s->b[i].z, &sx, &sy, &d)) d = 1e8f;
        depth[i] = d;
    }
    for (i = 1; i < n; i++) {
        int k = ord[i]; float d = depth[k];
        j = i;
        while (j > 0 && depth[ord[j - 1]] < d) { ord[j] = ord[j - 1]; j--; }
        ord[j] = k;
    }
    /* shadows first (far already sorted) */
    for (i = n - 1; i >= 0; i--) {
        const SimBody *b = &s->b[ord[i]];
        int sx, sy; float d;
        if (!project(s, b->x, 0.02f, b->z, &sx, &sy, &d)) continue;
        {
            float lift = b->y - b->hy;
            if (lift < 0.f)
                lift = 0.f;
            disc(sx, sy, (int)(s->foc * b->r * (0.45f + 0.12f * lift) / d),
                 mixfog(rgb565(16, 18, 14), d + lift * 2.f));
        }
    }
    for (i = 0; i < n; i++) {
        const SimBody *b = &s->b[ord[i]];
        int sx, sy; float d;
        if (!project(s, b->x, b->y, b->z, &sx, &sy, &d)) continue;
        if (b->shape == SIM_SPHERE) {
            int r = (int)(s->foc * b->r / d);
            disc(sx, sy, r, mixfog(pal(b->col, 88), d));
            disc(sx - r / 4, sy - r / 4, r / 3, mixfog(pal(b->col, 120), d));
        } else
            draw_box(s, b, d);
    }
    fill(0, 0, (int)FB_W, 16, rgb565(8, 10, 18));
    fill(0, (int)FB_H - 14, (int)FB_W, 14, rgb565(8, 10, 18));
    text(4, 4, "cSim2D", rgb565(220, 190, 70));
    snprintf(buf, sizeof buf, "n=%d j=%d f=%.0f", s->n, s->nj, s->foc);
    text(70, 4, buf, rgb565(200, 200, 210));
    text(4, (int)FB_H - 11, "WASD  QE yaw  space  +/- zoom  R  X", rgb565(140, 145, 160));
    fb_flip();
}
