#ifndef CSIM2D_H
#define CSIM2D_H

enum { SIM_MAX = 64, SIM_CIRCLE = 0, SIM_BOX = 1 };

typedef struct {
    float x, y, vx, vy, w;
    float m, im, I, iI;
    float r, hw, hh; /* circle r or box half */
    int shape, static_;
    int col;
} SimBody;

typedef struct {
    float gx, gy;
    float dt;
    float camx, camy, zoom, pitch;
    SimBody b[SIM_MAX];
    int n;
} Sim;

void sim_init(Sim *s);
int sim_add_circle(Sim *s, float x, float y, float r, float m);
int sim_add_box(Sim *s, float x, float y, float w, float h, float m);
void sim_static(Sim *s, int id);
void sim_kick(Sim *s, int id, float fx, float fy);
void sim_step(Sim *s, float dt);
void sim_draw(const Sim *s);
void sim_cam_follow(Sim *s, int id);

#endif
