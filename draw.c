#include "sim.h"
#include "fb.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
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
    int t[][3] = {{190,75,60},{60,130,200},{70,165,85},{210,175,55},{150,95,185},{210,115,45},{100,105,115},{220,220,230}};
    int i = abs(col) % 8;
    int r = t[i][0]*lit/100, g = t[i][1]*lit/100, b = t[i][2]*lit/100;
    if (r>255) r=255; if (g>255) g=255; if (b>255) b=255;
    return rgb565(r,g,b);
}
static void floor_mode7(const Sim *s)
{
    int y, x, hy = (int)FB_H / 2 + 8;
    float cy = cosf(s->yaw), syw = sinf(s->yaw);
    for (y = hy; y < (int)FB_H; y++) {
        float p = (float)(y - hy + 1) / (float)FB_H;
        float dist = s->camy / (p * 1.35f + 0.02f);
        float step = dist / s->foc;
        float wx0 = s->camx + syw * dist - cy * step * (float)FB_W * 0.5f;
        float wz0 = s->camz + cy * dist + syw * step * (float)FB_W * 0.5f;
        float dx = cy * step, dz = -syw * step;
        for (x = 0; x < (int)FB_W; x++) {
            int ix = (int)floorf(wx0), iz = (int)floorf(wz0);
            int c = (ix + iz) & 1;
            int fade = (int)(80.f + 40.f / (1.f + dist * 0.08f));
            px(x, y, c ? rgb565(fade/3, fade/2, fade/4) : rgb565(fade/4, fade/3, fade/5));
            wx0 += dx; wz0 += dz;
        }
    }
}
static void disc(int cx, int cy, int r, uint16_t c, uint16_t hi)
{
    int y, x;
    if (r < 1) r = 1;
    if (r > 80) r = 80;
    for (y = -r; y <= r; y++) {
        int w = (int)sqrtf((float)(r*r - y*y));
        for (x = -w; x <= w; x++) {
            float nx = x/(float)r, ny = y/(float)r;
            float nz = 1.f - nx*nx - ny*ny;
            px(cx+x, cy+y, (nz > 0.45f && nx < -0.1f && ny < -0.1f) ? hi : c);
        }
    }
}
static void draw_box(const Sim *s, const SimBody *b)
{
    float hx=b->hx, hy=b->hy, hz=b->hz;
    float P[8][3] = {
        {b->x-hx,b->y-hy,b->z-hz},{b->x+hx,b->y-hy,b->z-hz},
        {b->x+hx,b->y-hy,b->z+hz},{b->x-hx,b->y-hy,b->z+hz},
        {b->x-hx,b->y+hy,b->z-hz},{b->x+hx,b->y+hy,b->z-hz},
        {b->x+hx,b->y+hy,b->z+hz},{b->x-hx,b->y+hy,b->z+hz}
    };
    int S[8][2], ok[8], i;
    int edges[12][2] = {{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
    float d; int minx=999,miny=999,maxx=-999,maxy=-999;
    for (i=0;i<8;i++) ok[i]=project(s,P[i][0],P[i][1],P[i][2],&S[i][0],&S[i][1],&d);
    for (i=0;i<8;i++) if (ok[i]) {
        if (S[i][0]<minx) minx=S[i][0]; if (S[i][0]>maxx) maxx=S[i][0];
        if (S[i][1]<miny) miny=S[i][1]; if (S[i][1]>maxy) maxy=S[i][1];
    }
    if (maxx<minx) return;
    if (maxx-minx>220) maxx=minx+220;
    if (maxy-miny>220) maxy=miny+220;
    fill(minx,miny,maxx-minx+1,maxy-miny+1, pal(b->col,72));
    for (i=0;i<12;i++) {
        int a=edges[i][0], c=edges[i][1];
        if (ok[a]&&ok[c]) line(S[a][0],S[a][1],S[c][0],S[c][1], pal(b->col,35));
    }
}
void sim_draw(const Sim *s)
{
    int i,j,n=s->n,ord[SIM_MAX];
    float depth[SIM_MAX];
    char buf[64];
    fb_clear(rgb565(12,16,28));
    fill(0,0,(int)FB_W,(int)FB_H/2+8, rgb565(16,20,34));
    floor_mode7(s);
    for (i=0;i<n;i++) {
        int sx,sy; float d=1e9f;
        ord[i]=i;
        if (!project(s,s->b[i].x,s->b[i].y,s->b[i].z,&sx,&sy,&d)) d=1e8f;
        depth[i]=d;
    }
    for (i=1;i<n;i++) {
        int k=ord[i]; float d=depth[k];
        j=i;
        while (j>0 && depth[ord[j-1]]<d) { ord[j]=ord[j-1]; j--; }
        ord[j]=k;
    }
    for (i=0;i<n;i++) {
        const SimBody *b=&s->b[ord[i]];
        int sx,sy; float d;
        if (!project(s,b->x,b->y,b->z,&sx,&sy,&d)) continue;
        if (b->shape==SIM_SPHERE) {
            int r=(int)(s->foc * b->r / d);
            disc(sx,sy,r, pal(b->col,88), pal(b->col,120));
        } else draw_box(s,b);
    }
    fill(0,0,(int)FB_W,16,rgb565(8,10,18));
    fill(0,(int)FB_H-14,(int)FB_W,14,rgb565(8,10,18));
    text(4,4,"cSim2D", rgb565(220,190,70));
    snprintf(buf,sizeof buf,"n=%d", s->n);
    text(70,4,buf,rgb565(200,200,210));
    text(4,(int)FB_H-11,"WASD move  QE yaw  space hop  R reset  X quit", rgb565(140,145,160));
    fb_flip();
}
