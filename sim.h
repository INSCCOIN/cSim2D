#ifndef CSIM2D_H
#define CSIM2D_H
enum { SIM_MAX = 48, SIM_SPHERE = 0, SIM_BOX = 1 };
typedef struct {
    float x, y, z;
    float vx, vy, vz;
    float m, im;
    float r, hx, hy, hz;
    int shape, static_, col;
} SimBody;
typedef struct {
    float gx, gy, gz;
    float camx, camy, camz, yaw, foc;
    SimBody b[SIM_MAX];
    int n;
} Sim;
void sim_init(Sim *s);
int sim_add_sphere(Sim *s, float x, float y, float z, float r, float m);
int sim_add_box(Sim *s, float x, float y, float z, float sx, float sy, float sz, float m);
void sim_static(Sim *s, int id);
void sim_kick(Sim *s, int id, float fx, float fy, float fz);
void sim_step(Sim *s, float dt);
void sim_draw(const Sim *s);
void sim_cam_follow(Sim *s, int id);
#endif
