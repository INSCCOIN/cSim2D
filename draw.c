#include "sim.h"
#include "fb.h"
#include <math.h>
#include <stdio.h>

static void project(const Sim *s, float x, float y, int *sx, int *sy)
{
    float dx = x - s->camx;
    float dy = y - s->camy;
    *sx = (int)FB_W / 2 + (int)((dx - dy * 0.25f) * s->zoom);
    *sy = (int)((float)FB_H * 0.70f) - (int)((dy * s->pitch + dx * 0.12f) * s->zoom);
}

static uint16_t shade(int col, int lit)
{
    int pal[][3] = {
        {180, 80, 70}, {70, 140, 200}, {80, 170, 90}, {200, 170, 60},
        {160, 100, 190}, {200, 120, 50}, {90, 90, 100},
    };
    int i = (col & 7) % 7;
    int r = pal[i][0] * lit / 100, g = pal[i][1] * lit / 100, b = pal[i][2] * lit / 100;
    return rgb565(r, g, b);
}

static void disc(int cx, int cy, int rx, int ry, uint16_t c)
{
    int y, x;
    if (rx < 1)
        rx = 1;
    if (ry < 1)
        ry = 1;
    for (y = -ry; y <= ry; y++) {
        int w = (int)(rx * sqrtf(1.f - (float)(y * y) / (float)(ry * ry + 1)));
        for (x = -w; x <= w; x++)
            px(cx + x, cy + y, c);
    }
}

static void box25(const Sim *s, const SimBody *b)
{
    int x0, y0, x1, y1, x2, y2, x3, y3;
    int lift = (int)(b->hh * s->zoom * 0.35f);
    project(s, b->x - b->hw, b->y - b->hh, &x0, &y0);
    project(s, b->x + b->hw, b->y - b->hh, &x1, &y1);
    project(s, b->x + b->hw, b->y + b->hh, &x2, &y2);
    project(s, b->x - b->hw, b->y + b->hh, &x3, &y3);
    /* side */
    fill((x0 < x1 ? x0 : x1), (y0 < y1 ? y0 : y1),
         abs(x1 - x0) + 1, lift + 4, shade(b->col, 55));
    /* top face */
    {
        int minx = x0, maxx = x0, miny = y0 - lift, maxy = y0 - lift, i;
        int xs[4] = {x0, x1, x2, x3};
        int ys[4] = {y0 - lift, y1 - lift, y2 - lift, y3 - lift};
        for (i = 1; i < 4; i++) {
            if (xs[i] < minx)
                minx = xs[i];
            if (xs[i] > maxx)
                maxx = xs[i];
            if (ys[i] < miny)
                miny = ys[i];
            if (ys[i] > maxy)
                maxy = ys[i];
        }
        fill(minx, miny, maxx - minx + 1, maxy - miny + 1, shade(b->col, 90));
        rect(minx, miny, maxx - minx + 1, maxy - miny + 1, shade(b->col, 40));
    }
}

void sim_draw(const Sim *s)
{
    int i, gx, gy;
    uint16_t sky = rgb565(18, 22, 36), dirt = rgb565(42, 48, 40);
    char buf[64];
    fb_clear(sky);
    /* ground grid in world y=0 plane, x across */
    for (gx = -20; gx <= 20; gx++) {
        int x0, y0, x1, y1;
        project(s, (float)gx, 0, &x0, &y0);
        project(s, (float)gx, 0.02f, &x1, &y1);
        line(x0, y0, x0 + 40, y0 + (int)(12 * s->pitch), dirt);
    }
    for (gy = 0; gy <= 8; gy++) {
        int x0, y0, x1, y1;
        project(s, -16, (float)gy * 0.0f + gy * 0.01f, &x0, &y0);
        project(s, 16, gy * 0.01f, &x1, &y1);
        line(x0, y0 + gy * 3, x1, y1 + gy * 3, rgb565(36, 40, 48));
    }
    /* far to near: sort by x+y crude */
    {
        int ord[SIM_MAX], n = s->n;
        for (i = 0; i < n; i++)
            ord[i] = i;
        for (i = 1; i < n; i++) {
            int j, k = ord[i];
            float d = s->b[k].x + s->b[k].y;
            j = i;
            while (j > 0 && s->b[ord[j - 1]].x + s->b[ord[j - 1]].y < d) {
                ord[j] = ord[j - 1];
                j--;
            }
            ord[j] = k;
        }
        for (i = 0; i < n; i++) {
            const SimBody *b = &s->b[ord[i]];
            int sx, sy;
            project(s, b->x, b->y, &sx, &sy);
            if (b->shape == SIM_CIRCLE) {
                int rx = (int)(b->r * s->zoom);
                int ry = (int)(b->r * s->zoom * s->pitch);
                disc(sx, sy, rx, ry < 2 ? 2 : ry, shade(b->col, 85));
                disc(sx - rx / 4, sy - ry / 4, rx / 3, ry / 3, shade(b->col, 110));
            } else
                box25(s, b);
        }
    }
    fill(0, 0, (int)FB_W, 16, rgb565(10, 12, 20));
    fill(0, (int)FB_H - 14, (int)FB_W, 14, rgb565(10, 12, 20));
    text(6, 4, "cSim2D", rgb565(200, 180, 80));
    snprintf(buf, sizeof buf, "bodies %d", s->n);
    text(80, 4, buf, rgb565(200, 200, 210));
    text(6, (int)FB_H - 11, "arrows kick  +/- zoom  Q", rgb565(140, 145, 160));
    fb_flip();
}
