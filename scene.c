#include "sim.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int sim_load(Sim *s, const char *path)
{
    FILE *f;
    char line[256], tok[32];
    sim_init(s);
    f = fopen(path, "r");
    if (!f)
        return 0;
    while (fgets(line, sizeof line, f)) {
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '#' || *p == '\n' || !*p) continue;
        if (sscanf(p, "%31s", tok) != 1) continue;
        if (strcmp(tok, "body") == 0) {
            char kind[16];
            float x, y, z;
            if (sscanf(p, "%*s %15s %f %f %f", kind, &x, &y, &z) < 4) continue;
            if (strcmp(kind, "sphere") == 0) {
                float r = 0.4f, m = 1.f;
                sscanf(p, "%*s %*s %*f %*f %*f r %f m %f", &r, &m);
                sim_add_sphere(s, x, y, z, r, m);
            } else {
                float sx = 0.7f, sy = 0.7f, sz = 0.7f, m = 1.f;
                sscanf(p, "%*s %*s %*f %*f %*f size %f %f %f m %f", &sx, &sy, &sz, &m);
                sim_add_box(s, x, y, z, sx, sy, sz, m);
            }
        } else if (strcmp(tok, "static") == 0) {
            int id = 0;
            sscanf(p, "%*s %d", &id);
            sim_static(s, id);
        } else if (strcmp(tok, "joint") == 0) {
            char k[16];
            int a, b;
            sscanf(p, "%*s %15s %d %d", k, &a, &b);
            if (strcmp(k, "dist") == 0) {
                float rest = 1.f;
                sscanf(p, "%*s %*s %*d %*d %f", &rest);
                sim_joint_dist(s, a, b, rest);
            } else if (strcmp(k, "hinge") == 0) {
                float x, y, z;
                sscanf(p, "%*s %*s %*d %*d %f %f %f", &x, &y, &z);
                sim_joint_hinge(s, a, b, x, y, z);
            } else if (strcmp(k, "slide") == 0) {
                float x, y, z;
                sscanf(p, "%*s %*s %*d %*d %f %f %f", &x, &y, &z);
                sim_joint_slide(s, a, b, x, y, z);
            }
        } else if (strcmp(tok, "cam") == 0)
            sscanf(p, "%*s %f %f %f", &s->camx, &s->camy, &s->camz);
        else if (strcmp(tok, "yaw") == 0)
            sscanf(p, "%*s %f", &s->yaw);
        else if (strcmp(tok, "foc") == 0)
            sscanf(p, "%*s %f", &s->foc);
        else if (strcmp(tok, "grav") == 0)
            sscanf(p, "%*s %f %f %f", &s->gx, &s->gy, &s->gz);
        else if (strcmp(tok, "ground") == 0)
            sscanf(p, "%*s %f %f", &s->ge, &s->gmu);
        else if (strcmp(tok, "layer") == 0) {
            int id = 0;
            unsigned L = 1, M = 0xffffffffu;
            sscanf(p, "%*s %d %u %u", &id, &L, &M);
            if (id >= 0 && id < s->n) {
                s->b[id].layer = L;
                s->b[id].mask = M;
            }
        }
    }
    fclose(f);
    return 1;
}
